# Rosserial Windows

### This step by step will show you how to configure and run the project Rosserial in windows platform
 
### Configuration in linux

First, in Linux install rosserial_windows:

```sh
sudo apt-get install ros-kinetic-rosserial-windows
sudo apt-get install ros-kinetic-rosserial-server
```

### Run application

In order for rosserial_windows to communicate with the ROS master, a server socket is needed to connect.

In Linux:

```sh
roscore
```

In a separete terminal, start the rosserial server

```sh
rosrun rosserial_server socket_node
```

Running Turtlesim.


In a third terminal, use rostopic to see what your app sends 

```sh
turtle1/cmd_vel
```

In windows:

In your windows machine and run the app. You should see command velocities start appearing in the rostopic terminal on your ROS master. 