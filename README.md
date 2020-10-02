# mbot_simulation_sa

MBot robot stable simulation of the system without arm.

Brought to you by:

- [Instituto Superior Tecnico, Lisboa](http://welcome.isr.tecnico.ulisboa.pt/)
- [Institute for Systems and Robotics (ISR)](http://welcome.isr.tecnico.ulisboa.pt/)
- [Laboratório de Robótica e Sistemas em Engenharia e Ciência (LARSyS)](http://larsys.pt/)
- [SocRob RoboCup team](http://socrob.isr.tecnico.ulisboa.pt)

This code was tested on Ubuntu 16.04 and ROS kinetic with Gazebo 7

This is how the simulation looks like:

![alt mbot_simulation](https://github.com/socrob/mbot_simulation_sa/blob/master/resources/mbot_simulator.png "MBot simulation")

Installations instructions
==========================

Clone and compile the code into your catkin workspace:

        cd ~/{your_workspace}/src
        git clone https://github.com/socrob/mbot_simulation_sa.git
        cd mbot_simulation_sa
        
If you are using another ROS distribution other than Kinetic, please edit line 9 of **repository.debs** for your distro (*DISTRO=[your_distro]*). This might not work as the code was developed and tested using ROS Kinetic. Then you can proceed:

        ./repository.debs
        catkin build
        source ~/.bashrc

Launch the robot in an empty world:

        roslaunch mbot_simulation robot.launch
        
You can change the simulation environment by changing in the robot.launch file the world name and path.


Enjoy!

If errors, please report bugs by using the issues in this repository
