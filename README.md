# Sensors_for_Unity_ROS2_Humble
 2D LiDAR and Kinect Sensor simulating for ROS2



### Wie die C# Klassen verwenden? In Unity
[Lidar2DSensorROS](./Lidar2DSensorROS.cs) und ROSVerbindungLidar müssen auf ein Objekt oder leeres Objekt welches als Sensor dienen sollen in Unity gezogen werden.
Am Besten auch noch ROS2ForUnity von Robotec in den Assets Ordner einbinden.

### In ROS2 Humble
Kamerabild Ordner in ```src``` in den ROS2 Humble Workspace kopieren

```
colcon build
source install/setup.bash
ros2 run Kamerabild lidar1
```

zum Visualisieren in ```rviz2``` die RVIZ-Datei öffnen.
