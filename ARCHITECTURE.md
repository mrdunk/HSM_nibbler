# HSM Nibble — Architecture Overview

A guide for new developers. Covers module layout, key classes, and the execution flow from user call to output path.

---

## Module Dependency Graph

```mermaid
graph TD
    USER["User / CAM host"] --> geometry
    USER --> dxf

    subgraph hsm_nibble
        geometry["geometry.py\n(Pocket, EntryCircle,\nBaseGeometry)"]
        path_planner["path_planner.py\n(PathPlanner)"]
        path_assembler["path_assembler.py\n(PathAssembler,\nsplit_line_by_poly)"]
        arc_fitter["arc_fitter.py\n(find_best_arc_distance,\narc_at_distance)"]
        voronoi_centers["voronoi_centers.py\n(VoronoiCenters,\nVoronoiGraph)"]
        arc_utils["arc_utils.py\n(ArcData, LineData,\nArcDir, MoveStyle,\nStartPointTactic)"]
        dxf["dxf.py\ndxf_to_polygon()"]
        helpers["helpers.py\nlogging"]
        debug["debug.py\nDisplay (matplotlib)"]
    end

    geometry --> path_planner
    geometry --> path_assembler
    geometry --> voronoi_centers
    geometry --> arc_utils
    geometry --> debug

    path_planner --> arc_fitter
    path_planner --> path_assembler
    path_planner --> voronoi_centers
    path_planner --> arc_utils

    path_assembler --> arc_utils

    arc_fitter --> arc_utils

    voronoi_centers --> helpers
    debug --> arc_utils
    debug --> voronoi_centers

    PYVORONOI[("pyvoronoi\n(C++ Boost Voronoi)")] --> voronoi_centers
    SHAPELY[("shapely\n(2D geometry)")] --> geometry
    SHAPELY --> path_planner
    SHAPELY --> path_assembler
    SHAPELY --> arc_fitter
    SHAPELY --> voronoi_centers

    style arc_utils fill:#1a7a3a,color:#ffffff
    style geometry fill:#0066cc,color:#ffffff
    style USER fill:#b38600,color:#ffffff
    style PYVORONOI fill:#a02020,color:#ffffff
    style SHAPELY fill:#a02020,color:#ffffff
```

**`arc_utils.py` is a leaf** — no internal imports, defines all shared data types.  
**`geometry.py` is the root** — the only module a user needs to import directly.

---

## Key Classes

```mermaid
classDiagram
    class Pocket {
        +polygon: Polygon
        +step: float
        +winding_dir: ArcDir
        +path: List[ArcData|LineData]
        +start_point: Point
        +max_starting_radius: float
        +starting_angle: float
        +voronoi: VoronoiCenters
        +calculate_path()
        +get_arcs(timeslice) generator
    }

    class EntryCircle {
        +center: Point
        +radius: float
        +start_angle: float
        +spiral()
        +circle()
    }

    class BaseGeometry {
        +polygon: Polygon
        +step: float
        +winding_dir: ArcDir
        +assembler: PathAssembler
        +calculated_area_total: Polygon
        +path: List
        +_filter_input_geometry()
        +_queue_arcs()
        +_flush_arc_queues()
    }

    class PathPlanner {
        +voronoi: VoronoiCenters
        +assembler: PathAssembler
        +step: float
        +last_circle: ArcData
        +visited_edges: Set
        +calculated_area_total: Polygon
        +generate_path(timeslice) generator
        -_select_next_vertex()
        -_merge_voronoi_edges()
        -_calculate_arc()
    }

    class PathAssembler {
        +path: List[ArcData|LineData]
        +last_arc: ArcData
        +cut_area_total: Polygon
        +pending_arc_queues: List
        +queue_arcs(new_arcs)
        +flush_arc_queues()
        +join_arcs(next_arc) List[LineData]
        -_arcs_to_path()
        -_check_queue_overlap()
    }

    class VoronoiCenters {
        +graph: VoronoiGraph
        +polygon: Polygon
        +start_point: Point
        +start_distance: float
        +max_starting_radius: float
        +distance_from_geom(point) float
        +widest_gap() Point, float
        +vertex_on_perimiter() Point
        +for_widest_start()$
        +for_perimeter_start()$
    }

    class VoronoiGraph {
        +edges: Dict
        +vertex_to_edges: Dict
        +store(edge)
        +remove(edge_index)
        +vertexes_to_edge(a, b)
    }

    class ArcData {
        <<NamedTuple>>
        +origin: Point
        +radius: float
        +start: Point
        +end: Point
        +span_angle: float
        +winding_dir: ArcDir
        +path: LineString
    }

    class LineData {
        <<NamedTuple>>
        +start: Point
        +end: Point
        +path: LineString
        +move_style: MoveStyle
    }

    class ArcDir {
        <<Enum>>
        CW
        CCW
        CLOSEST
    }

    class MoveStyle {
        <<Enum>>
        CUT
        RAPID_INSIDE
        RAPID_OUTSIDE
    }

    class StartPointTactic {
        <<Enum>>
        PERIMETER
        WIDEST
    }

    BaseGeometry <|-- Pocket
    BaseGeometry <|-- EntryCircle
    Pocket --> PathPlanner : creates
    Pocket --> VoronoiCenters : creates / receives
    Pocket --> EntryCircle : creates
    BaseGeometry --> PathAssembler : owns
    PathPlanner --> VoronoiCenters : reads graph from
    PathPlanner --> PathAssembler : appends arcs to
    VoronoiCenters --> VoronoiGraph : owns
    PathAssembler --> ArcData : emits
    PathAssembler --> LineData : emits
```

---

## Execution Flow

```mermaid
sequenceDiagram
    participant User
    participant Pocket
    participant VoronoiCenters
    participant EntryCircle
    participant PathPlanner
    participant ArcFitter as arc_fitter
    participant PathAssembler

    User->>Pocket: Pocket(polygon, step, winding_dir, ...)
    Pocket->>VoronoiCenters: compute Voronoi diagram
    VoronoiCenters-->>Pocket: graph + start_point

    Pocket->>PathAssembler: create (with starting cut area)
    Pocket->>EntryCircle: create (entry helix)
    EntryCircle->>PathAssembler: spiral() + circle() arcs

    Pocket->>PathPlanner: create (voronoi, assembler, calculated area)
    Pocket->>PathPlanner: generate_path()

    loop Each unvisited Voronoi branch
        PathPlanner->>VoronoiCenters: _merge_voronoi_edges()
        VoronoiCenters-->>PathPlanner: merged edge LineString

        loop Along edge until end
            PathPlanner->>ArcFitter: find_best_arc_distance(edge, step, ...)
            ArcFitter-->>PathPlanner: (distance, circle, iteration_count)

            PathPlanner->>PathPlanner: split_arcs (clip to unplanned region)
            PathPlanner->>PathPlanner: filter_arc (drop trivial arcs)
            PathPlanner->>PathAssembler: queue_arcs(arcs)
            PathAssembler->>PathAssembler: assign to branch queue
            PathAssembler->>PathAssembler: flush ready queues → join_arcs()
            PathAssembler->>PathAssembler: append ArcData + LineData to path
        end
    end

    PathPlanner->>PathAssembler: flush_arc_queues() (drain remainder)
    Pocket-->>User: path = List[ArcData | LineData]
```

---

## Data Types in the Output Path

The `Pocket.path` list is the sole output. It contains two types of elements, interleaved:

```
path = [
    ArcData(origin, radius, start, end, span_angle, winding_dir, path),  ← tool cuts along arc
    LineData(start, end, path, move_style=CUT),                           ← short connecting cut
    ArcData(...),
    LineData(..., move_style=RAPID_INSIDE),                               ← rapid over already-cut area
    ArcData(...),
    LineData(..., move_style=RAPID_OUTSIDE),                              ← retract/rapid outside part
    ...
]
```

| Type | `move_style` / `winding_dir` | Meaning |
|---|---|---|
| `ArcData` | `CW` or `CCW` | Cut arc; tool follows arc path at feed rate |
| `LineData` | `CUT` | Short connecting move at feed rate |
| `LineData` | `RAPID_INSIDE` | Fast move over already-cut region (no material) |
| `LineData` | `RAPID_OUTSIDE` | Fast move; retract above part first |

---

## Arc Fitter: Bisection Algorithm

`find_best_arc_distance()` in `arc_fitter.py` is the core mathematical kernel.  
It places arcs so the gap between consecutive arcs equals the desired `step`.

```
Two spacing metrics depending on context:
  • _area_spacing      — new branch: place arc step away from already-cut boundary
  • _hausdorff_spacing — continuing: place arc so Hausdorff distance from prev arc ≈ step

Bisection approach:
  lo = current position   hi = edge end
  repeat up to 30 times:
      mid = (lo + hi) / 2
      if spacing(mid) < step:  lo = mid   ← too close, move outward
      else:                    hi = mid   ← too far, move inward
  return best_distance when |spacing - step| < step/20
```

---

## Typical User Code

```python
import ezdxf
from hsm_nibble import dxf, geometry
from hsm_nibble.arc_utils import ArcData, LineData, ArcDir, MoveStyle

# 1. Load geometry from a DXF file
dxf_data = ezdxf.readfile("part.dxf")
shape = dxf.dxf_to_polygon(dxf_data.modelspace())

# 2. Generate the toolpath
toolpath = geometry.Pocket(
    shape,
    step=1.0,             # radial spacing between arcs (mm)
    winding_dir=ArcDir.CW,
    generate=True,        # stream results incrementally
)

# 3. (optional) stream progress while generating
for progress in toolpath.get_arcs(timeslice=500):
    print(f"{progress:.0%}")

# 4. Consume the output path
for element in toolpath.path:
    if isinstance(element, ArcData):
        # arc cut: use origin, radius, start/end angles, winding_dir
        pass
    elif isinstance(element, LineData):
        if element.move_style == MoveStyle.RAPID_OUTSIDE:
            # retract, rapid, plunge
            pass
        else:
            # in-pocket move
            pass
```
