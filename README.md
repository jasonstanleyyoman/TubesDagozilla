# TubesDagozilla
**Simulate Fetching Ball**
```sh
roslaunch robot_gazebo field.launch
```
Command diatas berfungsi untuk memunculkan field di gazebo. Launch file di atas terdapat 3 turtlebot model burger, lapangan
yang sudah berstandar ssl, dan bola

Untuk mengambil bola, masukkan command ini:
```sh
rosrun robot_control fetch_ball.py Robot1
rosrun robot_control fetch_ball.py Robot2
rosrun robot_control fetch_ball.py Robot3
```
Masukkan salah satu command untuk memerintahkan robot 1,2,atau 3 untuk mengambil bola
