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
python3 -m unittest discover tests/
```

**Run a single test file:**
```
python3 -m unittest tests/test_arcs.py
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

**Run integration/coverage tests** (slower, runs against all `.dxf` test cases):
```
./tests/test.py
python3 -m unittest tests/test_coverage.py
python3 -m unittest tests/test_coverage_outer_peel.py
```

**Run example scripts** (requires a `.dxf` file from `test_cases/`):
```
./examples/example_basic.py ./test_cases/stegasorus.dxf
./examples/example_with_predrill.py ./test_cases/triceratops.dxf
./examples/example_outer.py ./test_cases/hextest.dxf
./examples/example_refine.py ./test_cases/thrash.dxf
```

## Architecture

### Core modules (`hsm_nibble/`)

- **`geometry.py`** — the main library. Contains the toolpath generation logic. Key types:
  - `ArcData` / `LineData` — NamedTuples representing moves in the output path
  - `ArcDir` (CW/CCW/Closest), `MoveStyle` (RAPID_OUTSIDE/RAPID_INSIDE/CUT), `StartPointTactic` (PERIMETER/WIDEST) — enums controlling path generation
  - Tuning constants at the top: `SKIP_EDGE_ARCS`, `CORNER_ZOOM`, `BREADTH_FIRST`, `ITERATION_COUNT`

- **`voronoi_centers.py`** — wraps the `pyvoronoi` package (which handles only straight lines/points) to compute Voronoi midpoints from polygon geometry that may contain arcs and circles. Prunes and simplifies the Voronoi diagram. Key class: `VoronoiCenters`.

- **`dxf.py`** — converts `.dxf` files into Shapely `Polygon` objects for use by the rest of the library.

- **`helpers.py`** — logging utilities.

- **`debug.py`** — `Display` class for optional matplotlib visualisation during development.

### Tests (`tests/`)

- `test_arcs.py` — unit tests for arc geometry (uses `unittest`)
- `test_voronoi.py` — unit tests for the Voronoi wrapper
- `test_coverage.py` / `test_coverage_outer_peel.py` — integration-style tests measuring toolpath coverage quality against `.dxf` test cases
- `test.py` — CLI script that runs all `.dxf` test cases and prints a coverage metrics table

### Key dependencies

- **`shapely`** (pinned `>=1.8.4,<1.9.0`) — 2D geometry operations throughout
- **`pyvoronoi`** — Voronoi diagram computation (wraps C++ Boost library)
- **`ezdxf`** — reading `.dxf` CAD files
