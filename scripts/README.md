# scripts/

## export_links_from_freecad.py

Export 17 link STL files (visual + collision) from the FreeCAD assembly.

### Usage

1. Open FreeCAD with the MCP RPC server running on port 9875
2. Import `step/robotdogassem.STEP` into a document named `RobotDog`
3. Execute this script via FreeCAD MCP `execute_code()` or paste into FreeCAD Python console

### Output

- `dog_robot_ws/src/dog_robot_description/meshes/visual/<link>.stl` (17 files)
- `dog_robot_ws/src/dog_robot_description/meshes/collision/<link>.stl` (17 files, convex hull)
