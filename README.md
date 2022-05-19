# <img alt="A Medusa stack for BlueROV" src="doc/img/logo/dsor_logo.jpg" height="60"> A Medusa stack for BlueROV
This repository holds the medusa code stack from DSOR-ISR (Dynamical Systems for Ocean Robotics - Institute for System Robotics) and additional software to simulate a BlueROV vehicle (regular or heavy configuration) or to serve a real BlueROV vehicle via the drivers code.

### Requirements
This code stack was developed with ROS1 in mind. In order to use, you are required to have:
- Ubuntu 20.04LTS (64-bit)
- ROS1 Noetic
- Python 3

### Installation

- Create a catkin workspace in the home directory, e.g.,
```
cd ~ && mkdir catkin_ws_bluerov && cd catkin_ws_bluerov
```

- Clone this repository and its submodules:
```
git clone --recursive git@github.com:dsor-isr/BlueROV_real.git src
```

- Run the requirements installation script:
```
bash ./install_requirements.sh
```

- Add the following lines at the end of your ~/.bashrc file:
```
# This is required (to source the ROS and medusa files)
source /usr/share/gazebo/setup.sh
source /usr/share/gazebo-11/setup.sh
source /opt/ros/noetic/setup.bash
export CATKIN_ROOT=$HOME
export ROS_WORKSPACE=${CATKIN_ROOT}/${CATKIN_PACKAGE}
export MEDUSA_SCRIPTS=$(find ${ROS_WORKSPACE}/src/ -type d -iname medusa_scripts | head -n 1)
source ${MEDUSA_SCRIPTS}/medusa_easy_alias/medusa_permanent_alias/alias.sh
```

- Compile the ROS packages (innside the catkin workspace folder)
```
catkin build
```

## Simulate the a vehicle and scenario w/ Gazebo

To start, we want to first create and simulate the scenario for the vehicle, via the following command

```
roslaunch bluerov_bringup start_scenario.launch
```

Then, after launching the scenario, one can "spawn" the BlueROV vehicle in the desired vheicle configuration (heavy or regular) via

```
roslaunch bluerov_bringup start_bluerov.launch vehicle_configuration:=<desired vehicle configuration>
```

### Some useful commands

There are some commands here and there that are common to be used, such as ```rostopic list```, to list all of the current topics available to the user, some provided by the Medusa stack, others by Gazebo, or even ROS. 

```rostopic pub -r10 /bluerov/ref/<surge, sway, etc> std_msgs/Float64 "data: 0.0" ``` provides a reference in sway/surge/etc according to the filled data (depending on the implemented inner loops). The -r10 specifies the rate of message publication since the vehicle inner loops only track references periodically.

```rostopic echo /bluerov/nav/filter/state``` outputs the current vehicle state in terms of current orientation, position (UTM), body_velocity, etc, given through the navigation filter (Kalman Filter). Even though there can be missing sensors to provide velocity (DVL), orientation (AHRS), etc, some of these data may be available nontheless through the implement filtering technique inmplemented by the navigation filter (dead-reckoning, etc).

And many more!

## Running the real vehicle
To have communication with the VectorNAV AHRS, you need to enable socat to pipe the data to the connected computer. For that, add
the following ALIAS to your bashrc and run the command before starting ROS.
```
alias vn100_socat="sudo socat pty,link=/dev/ttyVUSB1,raw,group-late=dialout,mode=660 tcp:192.168.2.2:8888"
```

### Commands that start automatically in raspberry pi
Add the 'socat_ahrs_bluerov.service' command in blueroses_addons to the raspberry pi's systemd.
```
sudo cp socat_ahrs_bluerov.service /etc/systemd/system
sudo systemctl enable socat_ahrs_bluerov.service
sudo systemctl start socat_ahrs_bluerov.service
```

### Accessing the video stream made via UDP
To access the video stream that is emitted by either the real or simulated BlueRov, please run the following line:
```
gst-launch-1.0  -v udpsrc port=5600 caps='application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264' ! rtph264depay ! avdec_h264 ! videoconvert ! autovideosink fps-update-interval=1000 sync=false
```

To access the video stream with OpenCV, refer to this example git repository only dedicated to this effect [github.com/bozkurthan/PX4-Gazebo-Opencv](https://github.com/bozkurthan/PX4-Gazebo-Opencv).

## Note:
At the moment, the gibmal of the bluerov vehicle is not controllable. 

### Install ROS on Raspberry pi running Raspian Buster OS
Check the online [install guide](https://varhowto.com/install-ros-noetic-raspberry-pi-4/).

### Bluerov Wifi
Disabling wifi in the real bluerov vehicle

```
/boot/config.txt
```

Added a line a the end to disable wifi - to re-enable, comment it

## Help and Support

- Official PX4 documentation for BlueROV: (docs.px4.io)[https://docs.px4.io/master/en/frames_sub/bluerov2.html]

- Examples on how to setup a simple BlueROV gazebo environment: 
[https://hippocampusrobotics.github.io/fav_docs/the_vehicle.html]
[https://hippocampusrobotics.github.io/fav_docs/next_steps/simulation.html]
[https://github.com/patrickelectric/bluerov_ros_playground]

### Active Developers
- João Quintas <jquintas@gmail.com>
- Marcelo Jacinto <marcelo.jacinto@tecnico.ulisboa.pt>
- André Potes <andre.potes@tecnico.ulisboa.pt>
- David Souto <david.souto@tecnico.ulisboa.pt>
- David Cabecinhas <dcabecinhas@isr.tecnico.ulisboa.pt>
- Bruno Cardeira <bcardeira@isr.tecnico.ulisboa.pt>

### Omnipresent
- Prof. António Pascoal
- Prof. Carlos Silvestre
- Prof. Rita Cunha
- Prof. Bruno Guerreiro
- Prof. Pedro Batista
- Luís Sebastião
- Manuel Rufino
- Pedro Gois
- Helena Santana