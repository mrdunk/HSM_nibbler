"""
Arc fitting logic: bisection-based convergence function.

These are extracted from Pocket._calculate_arc so the core algorithm can be
tested in isolation without constructing a full Pocket instance.
"""

from typing import Callable, List, Optional, Tuple

from shapely.geometry import LineString, Point, Polygon  # type: ignore

from hsm_nibble.arc_utils import (
    ArcData, ArcDir,
    create_circle, split_arcs,
)

# Kept for API compatibility with existing tests and callers.
# The bisection algorithm does not use it but accepts it as an ignored parameter.
ITERATION_COUNT = 50

# When arc sizes drop below a certain point, reduce the step size so the
# distance between arcs doesn't exceed the arc diameter.
# Minimum arc radius as a multiple of overlap at which step reduction begins.
CORNER_ZOOM = 2.0
# How much effect the feature has. 1 = step proportional to arc size.
CORNER_ZOOM_EFFECT = 1.0

# Bisection converges to desired_step / CONVERGE_FRACTION accuracy.
# 30 iterations on a 10-unit bracket gives ~1e-8 precision.
_BISECT_ITERATIONS = 30
_CONVERGE_FRACTION = 20  # stop when |spacing - desired| < desired / 20


class ProportionalController:
    """Kept for API compatibility. Not used by find_best_arc_distance."""

    def __init__(self, kp: float = 0.76) -> None:
        self._kp = kp

    def step(self, target: float, current: float) -> float:
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


def _desired_step(radius: float, step: float) -> float:
    """Return the corner-zoom-adjusted desired step for an arc of the given radius."""
    threshold = CORNER_ZOOM * step
    if radius < threshold:
        multiplier = (threshold - radius) / threshold
        return step * (1 - CORNER_ZOOM_EFFECT * multiplier)
    return step


def _hausdorff_spacing(
        d: float,
        voronoi_edge: LineString,
        winding_dir: ArcDir,
        calculated_area: Polygon,
        last_circle: ArcData,
        distance_from_geom: Callable[[Point], float],
        max_dist: float,
) -> Tuple[Optional[float], Optional[ArcData]]:
    """
    Return (spacing, circle) where spacing is the Hausdorff-based step from
    last_circle to the visible arcs at distance d, or None if the arc is
    entirely hidden.
    """
    pos, radius = arc_at_distance(d, voronoi_edge, distance_from_geom)
    circle = create_circle(pos, radius, winding_dir)
    arcs = split_arcs([circle], calculated_area)
    if not arcs:
        return None, circle
    spacing = -max_dist
    for arc in arcs:
        spacing = max(
            spacing,
            last_circle.origin.hausdorff_distance(arc.path) - last_circle.radius)
    return abs(spacing), circle


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
        controller: Optional[ProportionalController] = None,
) -> Tuple[float, ArcData, bool, int, bool]:
    """
    Find the distance along voronoi_edge at which an arc of the correct step
    size fits, using bisection on the Hausdorff spacing between arcs.

    Args:
        voronoi_edge: The line of midpoints between pocket edges.
        start_distance: Distance along voronoi_edge to start searching from.
        min_distance: Unused; kept for API compatibility.
        step: Desired distance between arc passes.
        winding_dir: CW or CCW winding direction for created circles.
        calculated_area: Area already planned — used to clip proposed arcs.
        last_circle: Reference circle for measuring step progress. None at the
            start of a new voronoi branch.
        distance_from_geom: Callable returning distance from a Point to the
            nearest pocket edge.
        max_dist: Maximum voronoi distance; used as a sentinel in spacing calc.
        controller: Ignored. Kept for API compatibility.

    Returns:
        (best_distance, best_circle, hidden_at_start, iteration_count, backwards)

        best_distance: Distance along voronoi_edge of the best-fit arc.
        best_circle: Full circle at best_distance (caller updates last_circle
            and calculated_area_total).
        hidden_at_start: True when the arc at the chosen position is entirely
            inside calculated_area. Caller should record best_circle as
            last_circle and return empty arcs without advancing.
        iteration_count: Number of bisection iterations used.
        backwards: Always False (bisection cannot move backward).
    """
    # ------------------------------------------------------------------
    # Case 1: start of a new voronoi branch — place at edge start.
    # ------------------------------------------------------------------
    if last_circle is None:
        pos, radius = arc_at_distance(start_distance, voronoi_edge, distance_from_geom)
        circle = create_circle(pos, radius, winding_dir)
        if split_arcs([circle], calculated_area):
            return (start_distance, circle, False, 1, False)
        # Edge start is inside already-cut area; advance by one step so the
        # caller can move dist forward and try again with last_circle set.
        d_next = min(start_distance + step, voronoi_edge.length)
        pos, radius = arc_at_distance(d_next, voronoi_edge, distance_from_geom)
        circle = create_circle(pos, radius, winding_dir)
        return (d_next, circle, True, 1, False)

    # ------------------------------------------------------------------
    # Case 2: near end of edge — place at edge end.
    # ------------------------------------------------------------------
    remaining = voronoi_edge.length - start_distance
    if remaining <= step:
        pos, radius = arc_at_distance(voronoi_edge.length, voronoi_edge, distance_from_geom)
        circle = create_circle(pos, radius, winding_dir)
        hidden = not split_arcs([circle], calculated_area)
        return (voronoi_edge.length, circle, hidden, 1, False)

    # ------------------------------------------------------------------
    # Case 3: bisect to find the distance where Hausdorff spacing = desired.
    #
    # lo=start_distance is always inside calculated_area (it's where the
    # previous arc was placed), so spacing there is 0 — no need to evaluate.
    # hi starts at start_distance + 2*step and expands to the edge end if
    # the arc at hi is hidden or spacing there is still below desired.
    # ------------------------------------------------------------------
    lo = start_distance
    hi = min(start_distance + 2 * step, voronoi_edge.length)

    sp_hi, _ = _hausdorff_spacing(
        hi, voronoi_edge, winding_dir, calculated_area, last_circle,
        distance_from_geom, max_dist)
    _, hi_radius = arc_at_distance(hi, voronoi_edge, distance_from_geom)
    desired_hi = _desired_step(hi_radius, step)

    # Expand hi to edge end if arc at hi is hidden or spacing is still low.
    if sp_hi is None or sp_hi < desired_hi:
        hi = voronoi_edge.length
        sp_hi, _ = _hausdorff_spacing(
            hi, voronoi_edge, winding_dir, calculated_area, last_circle,
            distance_from_geom, max_dist)
        _, hi_radius = arc_at_distance(hi, voronoi_edge, distance_from_geom)
        desired_hi = _desired_step(hi_radius, step)

    # If even the edge end can't reach desired spacing, place there and return.
    if sp_hi is None or sp_hi < desired_hi:
        pos, radius = arc_at_distance(voronoi_edge.length, voronoi_edge, distance_from_geom)
        circle = create_circle(pos, radius, winding_dir)
        hidden = not split_arcs([circle], calculated_area)
        return (voronoi_edge.length, circle, hidden, 1, False)

    # Bisect. Hidden arcs (sp_mid is None) are treated as spacing=0 → push lo up.
    best_circle: Optional[ArcData] = None
    count = 0
    for count in range(1, _BISECT_ITERATIONS + 1):
        mid = (lo + hi) / 2
        sp_mid, circle = _hausdorff_spacing(
            mid, voronoi_edge, winding_dir, calculated_area, last_circle,
            distance_from_geom, max_dist)

        if sp_mid is None:
            lo = mid
            continue

        _, mid_radius = arc_at_distance(mid, voronoi_edge, distance_from_geom)
        desired = _desired_step(mid_radius, step)

        if sp_mid < desired:
            lo = mid
        else:
            hi = mid
            best_circle = circle

        if abs(sp_mid - desired) < desired / _CONVERGE_FRACTION:
            break

    best_distance = min(hi, voronoi_edge.length)
    if best_circle is None:
        pos, radius = arc_at_distance(best_distance, voronoi_edge, distance_from_geom)
        best_circle = create_circle(pos, radius, winding_dir)

    hidden = not split_arcs([best_circle], calculated_area)
    return (best_distance, best_circle, hidden, count, False)
