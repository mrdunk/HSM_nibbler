# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this project is

`hsm_nibble` is a Python library that generates HSM (High Speed Machining) style CNC tool paths for pocket milling. It is a library — not a gcode generator. The tool path consists of a series of arcs, each centered on a Voronoi midpoint equidistant to two or more pocket edges, with radius set so the pocket edge is tangent to the arc. This minimises abrupt changes in tool engagement.

## Commands

**Install dependencies:**
```
pip install -r requirements.txt
```

**Run all unit tests:**
```
python3 -m unittest discover tests/unit/
```

**Run a single test file:**
```
python3 -m unittest tests/unit/test_arcs.py
```

**Run smoke tests** (full planner, fast):
```
python3 -m unittest tests/integration/test_smoke.py tests/integration/test_smoke_hard.py
```

**Run integration/coverage tests** (slower, runs against all `.dxf` test cases):
```
python3 -m unittest tests/integration/test_coverage.py
python3 -m unittest tests/integration/test_coverage_outer_peel.py
```

**Type checking:**
```
mypy hsm_nibble/
```

**Build and publish package:**
```
python3 -m build
python3 -m twine upload dist/*
```

**Run example scripts** (requires a `.dxf` file from `test_cases/`):
```
./examples/example_basic.py ./test_cases/stegasorus.dxf
./examples/example_with_predrill.py ./test_cases/triceratops.dxf
./examples/example_outer.py ./test_cases/hextest.dxf
./examples/example_refine.py ./test_cases/thrash.dxf
```

## Architecture

See [ARCHITECTURE.md](ARCHITECTURE.md) for module dependency graph, class diagram, and execution sequence.

### Core modules (`hsm_nibble/`)

- **`geometry.py`** — orchestrator and main entry point. Classes: `Pocket`, `EntryCircle`, `BaseGeometry`. Tuning constant: `SKIP_EDGE_ARCS`.

- **`arc_utils.py`** — leaf module (no internal imports). Defines all shared data types:
  - `ArcData`, `LineData` — NamedTuples representing moves in the output path
  - `ArcDir` (CW/CCW/CLOSEST), `MoveStyle` (RAPID_OUTSIDE/RAPID_INSIDE/CUT), `StartPointTactic` (PERIMETER/WIDEST) — enums controlling path generation
  - Low-level arc construction functions: `create_arc`, `create_circle`, `complete_arc`, `split_arcs`, `filter_arc`

- **`arc_fitter.py`** — bisection solver. `find_best_arc_distance()` places arcs at the correct step interval along a Voronoi edge using Hausdorff distance as the spacing metric.

- **`path_planner.py`** — Voronoi graph traversal. `PathPlanner` walks the Voronoi edges, calls `arc_fitter` for each position, and delegates output to `PathAssembler`.

- **`path_assembler.py`** — output path construction. `PathAssembler` buffers arcs in per-branch queues, inserts connecting moves (cut/rapid-inside/rapid-outside), and tracks the swept cut area. Also contains `split_line_by_poly()`.

- **`voronoi_centers.py`** — wraps the `pyvoronoi` package (which handles only straight lines/points) to compute Voronoi midpoints from polygon geometry that may contain arcs and circles. Prunes and simplifies the Voronoi diagram. Classes: `VoronoiCenters`, `VoronoiGraph`.

- **`dxf.py`** — converts `.dxf` files into Shapely `Polygon` objects.

- **`helpers.py`** — logging utilities.

- **`debug.py`** — `Display` class for optional matplotlib visualisation during development.

### Tests (`tests/`)

**`tests/unit/`** — fast, isolated, no DXF files required:
- `test_arcs.py` — arc geometry (original)
- `test_voronoi.py`, `test_voronoi_private.py` — Voronoi wrapper
- `test_arc_fitter.py`, `test_arc_fitter_private.py` — arc fitter public and private methods
- `test_arc_fitter_convergence.py` — replay captured bisection failure fixtures
- `test_path_assembler.py`, `test_path_assembler_private.py` — PathAssembler
- `test_path_planner.py`, `test_path_planner_extra.py` — PathPlanner
- `test_geometry.py` — BaseGeometry, EntryCircle, Pocket

**`tests/integration/`** — slower, requires `.dxf` files from `test_cases/`:
- `test_smoke.py`, `test_smoke_hard.py` — quick sanity checks against real DXF files
- `test_coverage.py`, `test_coverage_outer_peel.py` — coverage quality regression tests
- `run_all.py` — CLI metrics runner (not a test suite); prints a table of quality metrics
- `show_area_diff.py` — visualiser: renders what was planned vs actually cut

**`tests/fixtures/`** — captured convergence-failure inputs for `test_arc_fitter_convergence.py`.

### Key dependencies

- **`shapely>=2.0, <=2.0.6`** — 2D geometry operations throughout
- **`pyvoronoi>=1.2.8, <=1.2.8`** — Voronoi diagram computation (wraps C++ Boost library)
- **`ezdxf>=1.3.4, <=1.4.3`** — reading `.dxf` CAD files
- **`tabulate>=0.9.0`** — table formatting in `run_all.py`
- **`matplotlib>=3.9.2`** — optional visualisation in `debug.py` and `show_area_diff.py`
