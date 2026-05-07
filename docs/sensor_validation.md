# Sensor Validation Procedure (A0-A4)

This project validates real sensor feedback using Arduino angle channels `A0-A4`.

## Preconditions

- Arduino connected and detected by `angle_reader.py`.
- ROS 2 environment sourced.
- `angles_cache.json` is writable in project root.

## Steps

1. Start angle cache reader:
   - `python3 angle_reader.py`
2. Ensure cache updates:
   - `ls -lh angles_cache.json`
3. Build and source ROS workspace:
   - `cd ros_ws && colcon build`
   - `source install/setup.bash`
4. Launch bringup with rqt:
   - `ros2 launch arm4_bringup arm4_bringup.launch.py rqt:=true rviz:=false`
5. In `rqt_plot` add:
   - `/arm4/angles/data[0]`
   - `/arm4/angles/data[1]`
   - `/arm4/angles/data[2]`
   - `/arm4/angles/data[3]`
   - `/arm4/angles/data[4]`
6. Move arm joints and verify graph updates in real time.

## Evidence to capture for report

- terminal with running `angle_reader.py`;
- terminal with running ROS launch;
- `rqt_plot` screen with live signal changes.
