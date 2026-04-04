"""
Arc fitting logic: proportional controller and pure convergence function.

These are extracted from Pocket._calculate_arc so the core algorithm can be
tested in isolation without constructing a full Pocket instance.
"""

from typing import Callable, List, Optional, Tuple

from shapely.geometry import LineString, Point, Polygon  # type: ignore

from hsm_nibble.arc_utils import (
    ArcData, ArcDir,
    create_circle, split_arcs,
)

# Number of tries before giving up and accepting the best arc found so far.
ITERATION_COUNT = 50

# When arc sizes drop below a certain point, reduce the step size so the
# distance between arcs doesn't exceed the arc diameter.
# Minimum arc radius as a multiple of overlap at which step reduction begins.
CORNER_ZOOM = 2.0
# How much effect the feature has. 1 = step proportional to arc size.
CORNER_ZOOM_EFFECT = 1.0


class ProportionalController:
    """
    Proportional controller used to converge on the correct distance along a
    voronoi edge that produces the desired step size between arcs.

    Usage:
        controller = ProportionalController(kp=0.76)
        delta = controller.step(target=desired_step, current=measured_step)
        distance += delta
    """

    def __init__(self, kp: float = 0.76) -> None:
        self._kp = kp

    def step(self, target: float, current: float) -> float:
        """Return a distance correction proportional to the error."""
        error = target - current
        return self._kp * error


def extrapolate_line(overshoot: float, line: LineString) -> LineString:
    """
    Extend a line at both ends in the same direction it points.
    Used to give the convergence loop room to overshoot without hitting the
    endpoint clamp of LineString.interpolate().
    """
    coord_0, coord_1 = line.coords[:2]
    coord_m2, coord_m1 = line.coords[-2:]
    ratio_begin = overshoot / LineString([coord_0, coord_1]).length
    ratio_end = overshoot / LineString([coord_m2, coord_m1]).length
    coord_begin = Point(
        coord_0[0] + (coord_0[0] - coord_1[0]) * ratio_begin,
        coord_0[1] + (coord_0[1] - coord_1[1]) * ratio_begin)
    coord_end = Point(
        coord_m1[0] + (coord_m1[0] - coord_m2[0]) * ratio_end,
        coord_m1[1] + (coord_m1[1] - coord_m2[1]) * ratio_end)
    return LineString([coord_begin] + list(line.coords) + [coord_end])


def arc_at_distance(
        distance: float,
        voronoi_edge: LineString,
        distance_from_geom: Callable[[Point], float],
        overshoot: float = 100000,
) -> Tuple[Point, float]:
    """
    Calculate the center point and radius of the largest arc that fits at a
    given distance along a voronoi edge.

    The voronoi edge is extrapolated by `overshoot` at each end so that
    callers can pass a raw distance without worrying about endpoint clamping.
    """
    edge_extended = extrapolate_line(overshoot, voronoi_edge)
    pos = edge_extended.interpolate(distance + overshoot)
    radius = distance_from_geom(pos)
    return (pos, radius)


def find_best_arc_distance(
        voronoi_edge: LineString,
        start_distance: float,
        min_distance: float,
        step: float,
        winding_dir: ArcDir,
        calculated_area: Polygon,
        last_circle: Optional[ArcData],
        distance_from_geom: Callable[[Point], float],
        max_dist: float,
        controller: ProportionalController,
) -> Tuple[float, ArcData, bool, int, bool]:
    """
    Find the distance along voronoi_edge at which an arc of the correct step
    size fits, using the proportional controller to converge iteratively.

    Args:
        voronoi_edge: The line of midpoints between pocket edges.
        start_distance: Distance along voronoi_edge to start searching from.
        min_distance: Do not return arcs below this distance (travelling backwards).
        step: Desired distance between arc passes.
        winding_dir: CW or CCW winding direction for created circles.
        calculated_area: Area already planned — used to clip proposed arcs.
        last_circle: Reference circle for measuring step progress. May be None
            at the start of a new path segment.
        distance_from_geom: Callable that returns the distance from a Point to
            the nearest pocket edge (i.e. voronoi.distance_from_geom).
        max_dist: Maximum possible distance in the voronoi diagram, used as a
            sentinel for the spacing calculation.
        controller: ProportionalController instance (caller may reuse across calls).

    Returns:
        (best_distance, best_circle, hidden_at_start)

        best_distance: Distance along voronoi_edge of the best-fit arc.
        best_circle: Full circle at best_distance (before splitting against
            calculated_area). The caller is responsible for updating
            last_circle and calculated_area_total with this value.
        hidden_at_start: True when the very first proposed arc is entirely
            inside calculated_area and no progress has been made. The caller
            should record best_circle as last_circle (position reference) and
            return empty arcs without advancing.
        iteration_count: Number of iterations used. Equals ITERATION_COUNT
            when convergence failed.
        backwards: True when the algorithm converged to a position behind
            min_distance (moving the wrong way). Caller should return
            (voronoi_edge.length, []) without updating any state.
    """
    desired_step = min(step, voronoi_edge.length - start_distance)
    distance = start_distance + desired_step
    corner_zoom_threshold = CORNER_ZOOM * step

    count: int = 0
    best_distance: float = 0.0
    best_progress: float = 0.0
    best_circle: Optional[ArcData] = None
    progress: float = 0.0

    while count <= ITERATION_COUNT:
        count += 1

        pos, radius = arc_at_distance(distance, voronoi_edge, distance_from_geom)
        circle = create_circle(pos, radius, winding_dir)

        arcs = split_arcs([circle], calculated_area)
        if not arcs:
            if best_progress > 0:
                # Made some progress but this position is hidden — accept best so far.
                count = ITERATION_COUNT
                break
            # No progress at all — arc at start position is entirely hidden.
            return (distance, circle, True, count, False)

        # Measure how far the proposed arc is from the previous one.
        if last_circle is not None:
            spacing = -max_dist
            for arc in arcs:
                spacing = max(
                    spacing,
                    last_circle.origin.hausdorff_distance(arc.path) - last_circle.radius)
            progress = abs(spacing)
        else:
            spacing = -1.0
            for arc in arcs:
                for coord in arc.path.coords:
                    spacing = max(spacing, calculated_area.distance(Point(coord)))
            progress = spacing

        desired_step = min(step, voronoi_edge.length - start_distance)
        if radius < corner_zoom_threshold:
            multiplier = (corner_zoom_threshold - radius) / corner_zoom_threshold
            desired_step = step * (1 - CORNER_ZOOM_EFFECT * multiplier)

        if abs(desired_step - progress) < abs(desired_step - best_progress):
            best_progress = progress
            best_distance = distance
            best_circle = circle

            if abs(desired_step - progress) < desired_step / 20:
                break

        distance += controller.step(desired_step, progress)

    if best_circle is None:
        # Loop ran but never improved — use whatever we have.
        best_circle = circle
        best_distance = distance

    if count == ITERATION_COUNT and distance < min_distance:
        # Moving backwards along the voronoi edge.
        return (voronoi_edge.length, best_circle, False, count, True)

    if best_distance > voronoi_edge.length:
        best_distance = voronoi_edge.length

    return (best_distance, best_circle, False, count, False)
