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
    
## Node list

- Add object layer
- Mission control
- Adaptive planner



