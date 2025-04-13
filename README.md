# vc-stereo-ros2

**Status:**  This software is under active development.  Please submit an [Issue](https://github.com/apl-ocean-engineering/vc-stereo-ros2/issues) with any questions.

This is a minimum-working-example ROS2 driver for two cameras published via `libargus`, the Image Signal Processor (ISP) SW/HW provided by the **Jetpack** OS for Jetson devices.  It uses EGL/CUDA to perform YUV -> RGB mapping.

We are developing on a Jetson Orin Nano development kit running [Jetpack 6.2](https://developer.nvidia.com/embedded/jetpack) with two IMX296-based [Vision Components](https://www.vision-components.com/en/) camera modules.   For our testing, we run ROS2 "humble" built from source with our [custom installer](https://gitlab.com/rsa-perception-sensor/trisect_environment/-/tree/jetpack-6.1?ref_type=heads).  However as Jetpack 6.2 is based on Ubuntu 22.04, it should work with the standard "humble" packages from the ROS archive.

This software is mostly generic, but contains a few customizations specific to our case:

* It contains constants specific to our cameras (sensor size, etc.)
* We use GPIOs on the Jetson Nano board for external triggering.   This code contains a simple hardware loop for triggering the cameras.


[[TOC]]

### Installation

Argus Stereo Sync requires ROS2 and the Jetson Multimedia API.

Follow these steps to create a ROS workspace:
```sh
$ mkdir -p ros_ws/src && cd ros_ws/src
$ git clone https://github.com/apl-ocean-engineering/vc-stereo-ros2.git
$ vcs import --skip-existing < vc-stereo-ros2/vc-stereo-ros2.repos
$ cd ..
$ colcon build
```

This will build the package. To run the stereo  node, type:
```sh
$ source install/setup.bash
$ ros2 launch vc_stereo_ros2 stereo.launch.xml
```

### License

This driver is based on, but signicantly evolved from [NeilKhera/argus_stereo_sync](https://github.com/NeilKhera/argus_stereo_sync) which is itself based on sample code from the Jetson Multimedia API.  It retains the MIT license of that original version.

> MIT License. Copyright (c) 2025 University of Washington

> Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

> The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

> THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
