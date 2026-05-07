# Final Report Template (Robot ARM4)

## 1. Project link
- GitHub repository: `<insert_link>`

## 2. Concept and architecture
- Brief concept of ARM4.
- System architecture diagram.
- Device and interface map.

## 3. Installation and environment setup
- Run `scripts/install_dependencies.sh`.
- Verify Python, pyserial, ROS.
- Screenshot(s).

## 4. SSH setup
- Run `scripts/setup_ssh.sh`.
- Verify SSH service status.
- Screenshot(s).

## 5. ROS bringup
- Build `ros_ws`.
- Launch with parameters:
  - `rviz:=true`
  - `rqt:=true`
- Screenshot(s).

## 6. Sensor/device validation
- Show A0-A4 channel data in `rqt_plot` or RViz.
- Screenshot(s) with short comments.

## 7. Docker
- Build image (`docker build -t arm4:latest .`).
- Run container and show ROS startup.
- Screenshot(s).

## 8. Final checklist
- All final assignment requirements satisfied.
- Short conclusion.
