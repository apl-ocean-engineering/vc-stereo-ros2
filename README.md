# Argus Stereo Sync

This a ROS2 driver for publishing data from two cameras through the `libargus`, the Image Signal Processor built into the **Jetpack** OS for Jetson devices.  We have been testing with two IMX296-based [Vision Components](https://www.vision-components.com/en/) cameras attached to a Jetson Orin Nano development kit running [Jetpack 6.2](https://developer.nvidia.com/embedded/jetpack).   As Jetpack 6.2 is based on Ubuntu 22.04, we run ROS2 "humble" built from source with our [custom installer](https://gitlab.com/rsa-perception-sensor/trisect_environment/-/tree/jetpack-6.1?ref_type=heads).

**Status:**  This software is under active development.  It contains constants specific to our cameras (sensor size, etc.) and to the Jetson Nano baseboard (GPIOs used for external triggering).

[[TOC]]

### Installation
> Argus Stereo Sync requires ROS2 and the Jetson Multimedia API:

```sh
$ sudo apt-get install -y nvidia-l4t-jetson-multimedia-api
```


Follow these steps to create a ROS workspace:
```sh
$ mkdir catkin_ws && cd catkin_ws
$ mkdir src && cd src
$ catkin_init_workspace
```
Install Argus Stereo Sync:
```sh
$ git clone https://github.com/Nekhera/argus_stereo_sync.git
$ cd argus_stereo_sync/libs
$ cd /path/to/catkin_ws
$ catkin_make -DCMAKE_BUILD_TYPE=Release
```

This will build the package. To run the stereo sync node, type:
```sh
$ ros2 launch argus_stereo_sync argus_stereo.launch.xml
```


### License

This driver is signicantly evolved from [NeilKhera/argus_stereo_sync](https://github.com/NeilKhera/argus_stereo_sync) and retains the MIT license of that original version.

> MIT License. Copyright (c) 2025 University of Washington

> Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

> The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

> THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
