# junbot_adaptive_planner
Adaptive planner for Junbot based on object mapping

## Addition dependencies for ros_motion_planning

- OSQP
    ```bash
    git clone -b release-0.6.3 --recursive https://github.com/oxfordcontrol/osqp
    cd osqp && mkdir build && cd build
    cmake .. -DBUILD_SHARED_LIBS=ON
    make -j6
    sudo make install
    sudo cp /usr/local/include/osqp/* /usr/local/include
    ```

- OSQP-Eigen

    ```bash
    git clone https://github.com/robotology/osqp-eigen.git
    cd osqp-eigen && mkdir build && cd build
    cmake ..
    make
    sudo make install
    ```

- Other dependence.
    ```bash
    sudo apt install python-is-python3 \
    nlohmann-json3-dev \
    ros-noetic-amcl \
    ros-noetic-base-local-planner \
    ros-noetic-map-server \
    ros-noetic-move-base \
    ros-noetic-vision-msgs 
    ```
## Addition dependencies for turtlebot

```bash
sudo apt-get install ros-noetic-joy ros-noetic-teleop-twist-joy \
  ros-noetic-teleop-twist-keyboard ros-noetic-laser-proc \
  ros-noetic-rgbd-launch ros-noetic-rosserial-arduino \
  ros-noetic-rosserial-python ros-noetic-rosserial-client \
  ros-noetic-rosserial-msgs ros-noetic-amcl ros-noetic-map-server \
  ros-noetic-move-base ros-noetic-urdf ros-noetic-xacro \
  ros-noetic-compressed-image-transport ros-noetic-rqt* ros-noetic-rviz \
  ros-noetic-gmapping ros-noetic-navigation ros-noetic-interactive-markers \
  ros-noetic-dynamixel-sdk ros-noetic-turtlebot3-msgs
```


export ROS_MASTER_URI=http://127.0.0.1:11311

export ROS_HOSTNAME=127.0.0.1

export ROS_IP=127.0.0.1

## Docker build

Download a cuDNN 8.6.0 installer deb-package from NVIDIA's download page (login required).

Place the cuDNN installer deb-package in this directory and rename it to cudnn-local-repo-$(dpkg --print-architecture).deb.

Download a TensorRT 8.5.1 installer deb-package from NVIDIA's download page (login required).

Place the TensorRT installer deb-package in this directory and rename it to nv-tensorrt-local-repo-$(dpkg --print-architecture).deb.

Build the Docker image with below command.

```bash
docker buildx build \
    --load \
    --platform $(uname)/$(uname -m) \
    --build-arg UBUNTU_VERSION=20.04 \
    --build-arg CUDA_VERSION=11.8 \
    --build-arg CUDNN_VERSION=8.6.0 \
    --build-arg TENSORRT_VERSION=8.5.1 \
    --build-arg INSTALL_AS_DEV=1 \
    --tag ate_junbot \
    .
```    
## Node list

- Add object layer
- Mission control
- Adaptive planner



