diff --git a//dev/null b/README.md
index 0000000000000000000000000000000000000000..362d8f980ce32b0dcbcc591e1b01508b8bc67b61 100644
--- a//dev/null
+++ b/README.md
@@ -0,0 +1,29 @@
+# ROS 2 Workspace
+
+This repository contains a set of ROS 2 packages for working with Dynamixel actuators and joystick input.
+
+## Packages
+- **command_dyposition** – TODO: Package description.
+- **dynamixel_sdk** – Wrapping version of ROBOTIS Dynamixel SDK for ROS 2 providing Dynamixel control functions.
+- **dynamixel_sdk_custom_interfaces** – ROS2 custom interface examples using ROBOTIS DYNAMIXEL SDK.
+- **dynamixel_sdk_examples** – ROS2 examples using ROBOTIS DYNAMIXEL SDK.
+- **exmple_dynamixel_cpp** – ROS2 node to control Dynamixel XM540-W270-R using Dynamixel Workbench Toolbox.
+- **joyserv** – TODO: Package description.
+- **joysub** – directory is present but does not contain a package.
+
+## Build Instructions
+1. Source your ROS 2 installation (replace `humble` with your distro if different).
+   ```bash
+   source /opt/ros/humble/setup.bash
+   ```
+2. From this workspace root, invoke `colcon` to build all packages:
+   ```bash
+   colcon build --symlink-install
+   ```
+3. After the build completes, source the workspace:
+   ```bash
+   source install/setup.bash
+   ```
+
+## License
+Licensing is provided per package. Please see each package's `LICENSE` file for specific terms.
