# A library for generating HSM tool paths for CNC pocket milling.

## TLDR
This project generates HSM (High Speed Machining) style CNC tool paths which
minimise abrupt changes in tool engagement.

It is intended to be used as a library in a CAM package.

It is NOT intended to be used to directly generate gcode.

See the "Projects using this library" section for CAM projects using this library
to generate gcode.

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

## How to use this library
### Basic calculation of a internal pocket:
```
$ ./examples/example_basic.py ./test_cases/stegasorus.dxf
```
### Calculation of a internal pocket starting from pre-drilled hole:
```
./examples/example_with_predrill.py ./test_cases/triceratops.dxf
```
### Calculation of an outside clearing operation where the edge of the stock to be cut is encoded as the outside polygon in the .dxf file:
```
examples/example_outer.py ./test_cases/hextest.dxf
```
### Calculation of an outside clearing operation where the edge of the stock to be cut is calculated as a simple offset from the outer edge of the .dxf part file.
```
examples/example_refine.py ./test_cases/thrash.dxf
```

## Other useful libraries in this repo
The VoronoiCenters class in hsm_nibble/voronoi_centers.py is a wrapper around the pyvoronoi package.
pyvoronoi only deals with straight lines and points. Not arcs.
The VoronoiCenters class prunes the voroni diagram and tries to do same things when simplifying a voronoi diagram created from geometry containing arc and circles.

## Projects using this library
- [DerpCAM](https://github.com/kfoltman/DerpCAM) A 2.5D CAM aimed at hobby CNC machine users.
