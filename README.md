# A library for generating HSM tool paths for CNC pocket milling.

## TLDR
This project generates HSM (High Speed Machining) style CNC tool paths which
minimise abrupt changes in tool engagement.

It is intended to be used as a library in a CAM package.

It is NOT intended to be used to directly generate gcode.

## The problem
When CNC milling pockets, calculating a tool path that minimises endmill engagement
and abrupt *changes* in engagement is challenging. There are few choices available to
hobbyists and even less open source options.

Contour milling is the standard way of milling pockets where successive cuts run
parallel to each other leading to high endmill engagement in tight corners and
abrupt changes in endmill engagement.

This project generates a tool path which consists of a series of arcs. Each arc is
centered on a point equidistant to two or more pockets edges and with an arc radius
set so the pocket edge is a tangent to the arc.
This leads to gradual increases in tool engagement, greatly decreasing the change
of endmill damage.

![Example path](/images/longneck.png)

## Projects using this library
- [DerpCAM](https://github.com/kfoltman/DerpCAM) A 2.5D CAM aimed at hobby CNC machine users.

---

## How to use this library

### Basic calculation of an internal pocket:
```
./examples/example_basic.py ./test_cases/stegasorus.dxf
```
### Calculation of an internal pocket starting from a pre-drilled hole:
```
./examples/example_with_predrill.py ./test_cases/triceratops.dxf
```
### Calculation of an outside clearing operation where the edge of the stock is encoded as the outside polygon in the .dxf file:
```
examples/example_outer.py ./test_cases/hextest.dxf
```
### Calculation of an outside clearing operation where the edge of the stock is calculated as a simple offset from the outer edge of the .dxf part file:
```
examples/example_refine.py ./test_cases/thrash.dxf
```

The main entry point is `geometry.Pocket`. A minimal integration looks like:

```python
import ezdxf
from hsm_nibble import dxf, geometry
from hsm_nibble.arc_utils import ArcData, LineData, ArcDir, MoveStyle

shape = dxf.dxf_to_polygon(ezdxf.readfile("part.dxf").modelspace())

toolpath = geometry.Pocket(shape, step=1.0, winding_dir=ArcDir.CW)

for element in toolpath.path:
    if isinstance(element, ArcData):
        # arc cut — use origin, radius, start/end angles, winding_dir
        pass
    elif isinstance(element, LineData):
        if element.move_style == MoveStyle.RAPID_OUTSIDE:
            pass  # retract, rapid, plunge
        else:
            pass  # in-pocket connecting move
```

### Output fields

`toolpath.path` is a `list` of `ArcData` and `LineData` elements (both are `NamedTuple`s).

**`ArcData`** — a cutting arc:

| Field | Type | Description |
|---|---|---|
| `origin` | `Point` | Arc centre |
| `radius` | `float` | Arc radius |
| `start` | `Point` | Start point on the arc |
| `end` | `Point` | End point on the arc |
| `start_angle` | `float` | Angle to `start` from centre (radians) |
| `span_angle` | `float` | Signed angular extent; negative = CW |
| `winding_dir` | `ArcDir` | `CW` or `CCW` |
| `path` | `LineString` | Discretised arc geometry (for geometry ops) |

**`LineData`** — a connecting move:

| Field | Type | Description |
|---|---|---|
| `start` | `Point` | Move start |
| `end` | `Point` | Move end |
| `path` | `LineString` | Line geometry |
| `move_style` | `MoveStyle` | `CUT`, `RAPID_INSIDE`, or `RAPID_OUTSIDE` |

`RAPID_OUTSIDE` means the tool must retract above the part before the move and
plunge again afterwards. `RAPID_INSIDE` is a fast move over already-cut material
with no retract needed. `CUT` is a short feed-rate connecting move.

### Streaming / progress reporting

For large or complex pockets, pass `generate=True` to stream arcs incrementally
rather than blocking until the full path is computed. `get_arcs(timeslice=N)`
yields a progress ratio (0.0–1.0) each time N milliseconds of computation have
elapsed, so a GUI host can stay responsive:

```python
toolpath = geometry.Pocket(shape, step=1.0, winding_dir=ArcDir.CW, generate=True)

for progress in toolpath.get_arcs(timeslice=200):
    print(f"{progress:.0%}")
    # toolpath.path contains all arcs computed so far
```

### Pre-drilled holes and multi-pass operations

If the tool has already cut some area (e.g. a pre-drilled entry hole, or a prior
roughing pass), pass it as `already_cut` so the planner avoids re-cutting it and
routes the entry correctly:

```python
from shapely.geometry import Point

entry_hole = Point(x, y).buffer(hole_radius)
toolpath = geometry.Pocket(shape, step=1.0, winding_dir=ArcDir.CW,
                           already_cut=entry_hole)
```

## Other useful libraries in this repo
The `VoronoiCenters` class in `hsm_nibble/voronoi_centers.py` is a wrapper around the [pyvoronoi](https://github.com/fabanc/pyvoronoi)
package.  
Unfortunately pyvoronoi only deals with straight lines and points. Not arcs.  
The `VoronoiCenters` class prunes the Voronoi diagram and tries to do the same things when simplifying a Voronoi diagram created from geometry containing arcs and circles, similar to what the [C++ Boost Voronoi library](https://www.boost.org/doc/libs/latest/libs/polygon/doc/voronoi_main.htm) does for C++.

---

## Architecture

See [ARCHITECTURE.md](ARCHITECTURE.md) for module dependency graph, class diagram, execution
sequence, and an explanation of the arc-fitting bisection algorithm.

---

## Testing

### Unit tests (`tests/unit/`)

Fast, isolated tests for individual functions and classes — no DXF files
required. Run them after any code change:

```
python3 -m unittest discover tests/unit/
```

### Smoke tests (`tests/integration/test_smoke.py`, `test_smoke_hard.py`)

Quick sanity checks that run the full path planner against one or two real DXF
files and assert high-level properties (non-empty path, no dangerous crashes,
low path-fail ratio). They are slower than unit tests but much faster than the
full coverage suite:

```
python3 -m unittest tests/integration/test_smoke.py tests/integration/test_smoke_hard.py
```

### Coverage / integration tests (`tests/integration/test_coverage.py`, `test_coverage_outer_peel.py`)

Run the planner across all `.dxf` test cases in `test_cases/` and measure how
much of the pocket area is actually cut. These tests catch regressions in
coverage quality but take significantly longer:

```
python3 -m unittest tests/integration/test_coverage.py
python3 -m unittest tests/integration/test_coverage_outer_peel.py
```

### Convergence regression fixtures (`tests/unit/test_arc_fitter_convergence.py`)

Replay captured inputs where the arc-fitting bisection previously failed to
converge. Ensures fixes stay fixed:

```
python3 -m unittest tests/unit/test_arc_fitter_convergence.py
```

### Metrics runner (`tests/integration/run_all.py`)

Not a test suite — a CLI script that runs the planner over all DXF test cases
and prints a table of quality metrics (uncut area, arc count, time per arc,
etc.). Useful for manual performance comparisons:

```
python3 tests/integration/run_all.py
python3 tests/integration/run_all.py ./test_cases/square_circle.dxf --overlap 1.6 --winding CW
```

### Area-diff visualiser (`tests/integration/show_area_diff.py`)

Renders a PNG (or interactive window) showing what the planner *thinks* was cut
versus what the output arcs actually cover. Useful for diagnosing coverage gaps:

```
python3 tests/integration/show_area_diff.py test_cases/arcs.dxf --outdir /tmp/HSM
python3 tests/integration/show_area_diff.py test_cases/arcs.dxf --interactive
```

---

## Linting

```
mypy ./hsm_nibble/
```

---

## Install dependencies

Requires Python 3.7+.

```
pip install -r requirements.txt
```
