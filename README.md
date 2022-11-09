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
See `demo.py` file.

## Projects using this library
- [DerpCAM](https://github.com/kfoltman/DerpCAM) A 2.5D CAM aimed at hobby CNC machine users.
