# Note for MIT Cheetah Software Installation

Ubuntu 18.04 LTS

## Dependency

- Qt
    Download [Qt Online Installer](https://www.qt.io/download-qt-installer).
    ```
    sudo chmod +X qt-unified-linux-x64-4.1.1-online.run
    ./qt-unified-linux-x64-4.1.1-online.run
    ```
    After registration, install **qt5.10.0** newer (requires the gamepad library).
    NOTE: on Ubuntu 18.10 or 19.04, you may instead install Qt with
    ```
    sudo apt install libqt5 libqt5gamepad5
    ```

- [LCM](https://lcm-proj.github.io/)
    Need Java preinstalled.
    ```
    git clone https://github.com/lcm-proj/lcm.git 
    cd lcm
    mkdir build && cd build 
    cmake .. 
    make 
    sudo make install 
    sudo ldconfig
    ```
- [Eigen](http://eigen.tuxfamily.org/)
    Install and copy `<eigen3>` to `<usr/local/include>`
    ```
    sudo apt install libeigen3-dev
    sudo cp -r /usr/include/eigen3  /usr/local/include
    ```
- qpOASES

- `mesa-common-dev`
- `freeglut3-dev`
- `libblas-dev liblapack-dev`

    ```
    sudo apt install mesa-common-dev freeglut3-dev coinor-libipopt-dev libblas-dev liblapack-dev gfortran liblapack-dev coinor-libipopt-dev cmake gcc build-essential libglib2.0-dev
    ```
## Build
- Follow the official guide.
    ```
    git clone https://github.com/mit-biomimetics/Cheetah-Software.git
    mkdir build
    cd build
    cmake ..
    ./../scripts/make_types.sh
    make -j4
    ```
- Run `./common/test-common` to test the common library.

## Simulation

1. Using LCM on a single host
    ```
    sudo ifconfig lo multicast
    sudo route add -net 224.0.0.0 netmask 240.0.0.0 dev lo
    ```
1. Open the simulation control board
    ```
    ./sim/sim
    ```
1. On the left panel, select `Mini Cheetah` and `Simulator` and click `Start`.
2. Check your controller is connected wired or wirelessly.
1. Set `use_rc` to 0 to turn off R/C radio remote control.
3. In the another terminal, run the robot control code
    ```
    ./user/${controller_folder}/${controller_name} ${robot_name} ${target_system}
    ```
    Example
    ```
    ./user/MIT_Controll/mit_ctrl m s
    ```
    3: Cheetah 3; m: Mini Cheetah
    s: simulation; r: robot
4. Set `control_mode` to 6 to stand up.
5. Set `control_mode` to 4 to trot.
6. Use your controller to move the robot.