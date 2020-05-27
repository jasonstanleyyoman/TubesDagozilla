# TubesDagozilla
**Prerequisite**
Turtlebot3 ros melodic
```sh
sudo apt-get install ros-melodic-turtlebot3-description
```
```sh
sudo apt-get install ros-melodic-turtlebot3-teleop
```

**Simulate Fetching Ball**
```sh
roslaunch robot_gazebo field.launch
```
Command diatas berfungsi untuk memunculkan field di gazebo. Launch file di atas terdapat 3 turtlebot model burger, lapangan
yang sudah berstandar ssl, dan bola

```sh
roscd robot_control/src
```
```sh
chmod +x fetch_ball.py
```


Untuk mengambil bola, masukkan command ini:
```sh
rosrun robot_control fetch_ball.py Robot1
rosrun robot_control fetch_ball.py Robot2
rosrun robot_control fetch_ball.py Robot3
```
Masukkan salah satu command untuk memerintahkan robot 1,2,atau 3 untuk mengambil bola
Gerakkan bola untuk melihat simulasi robot mengejar bola

**Simulate defending**
```sh
roslaunch robot_gazebo defending.launch
```

```sh
roscd robot_control/src
```
```sh
chmod +x defending.py
```
```sh
rosrun robot_control defending.py
```
```sh
rosrun turtlebot3_teleop turtlebot3_teleop_key /cmd_vel:=/robot2/cmd_vel
```
Gerakkan robot2 melalui console ini untuk melihat simulasi defending
