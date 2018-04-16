# BURobotics_UWSim

This repo has 4 folders that go in four seperate folders in the ROS structure. 
_________________________________________________________________________________
The NEXXUS_ROV folder houses the URDF file for the ROV. It should be put in two places. The first is:

```~/opt/ros/kinetic/share/urdf_tutorial``` 

folder. This will allow ros to access it from "anywhere."

The second place it should be put is 

```~/catkin_ws/src```
___________________________________________________________________________________
The Kinematic_Scenes file holds files that should be put in the:

```~/catkin_ws/src/underwater_simulation/uwsim/data/scenes```

and

```~catkin_ws/install_isolated/share/uwsim/data/scenes``` 

folders. Putting them in the src allows them to be compiled but we have been editing the install file directly so the most recent simulation will be able to be edited there. It may be useful to update the xacro file later. 

_______________________________________________________________________________
The Experimental_Physics_Scenes folder should be left outside the catkin_ws. It will not complie without the other physics modules - which will be coming soon. But it contains another version of the pipeFollowing_turnsNEXXUS.xml main scene file that contains physics and mass on each object. 

______________________________________________________________________________________
The uw_teleop folder should go in:

```~/catkin_ws/src```

It is a full ROS package. It assumes that the ROS package joy (```http://wiki.ros.org/joy```) is running a joy_node, and a Logitech X3D joystick is connected on ubs/js0. The 3 programs in the /scripts folder can be run with python to control the ROV and arms with the joystick. It also contains the experimental physics script for controlling the ROV with thrusters (and requires the not included physics file). 

_________________________________________________________________________________

Once the files have been put in the appropriate locations, you will have to go through the file pipeFollowing_turnsNEXXUS.xml and change all of the file paths to reflect where the respective files are in your computer. In general you will just need to change /home/burobotics to whatever your computer is. 

To run the program, catkin_make_isolated the /catkin_ws and then source the devel_isolated/setup.bash file. 

The code to run the uwsim is 
```roscore```
*New Terminal Window*
```rosrun uwsim uwsim --configfile pipeFollowing_turnsNEXXUS.xml```

You will also need to run the joy_node package and the uw_teleop python scripts to move things around. 

Email me if anything doesnt work or you want to change anything in the simulation.
