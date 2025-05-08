# Kinect Sensor-Modell in Unity implementieren und in RVIZ ausführen

### C# Klassen in Unity
[KinectSensor](KinectSensor.cs) auf ein leeres Objekt als Komponente hinzufügen
Diesem leeren Objekt gehören nun einige Objekte zugeordnet (im Inspector rechts):
	DepthCamera gehört eine Camera zugeordnet z.B. "DepthCam" 
	RGBCamera gehört eine extra Camera zugeordnet z.B. "RGBCam"
	
Wichtig: Es müssen zwei unterschiedliche Kameras sein, sonst funktioniert das System nicht.
	 Zudem müssen Parameter wie z.B. Blickfeld selbst eingestellt werden...
		 
Dem DepthMaterial muss DepthMT zugewiesen werden



### ROS2 Humble Paket: ```kinect_unity_empfaenger```
Das Paket [kinect_unity_empfaenger](./) in den ```src``` Ordner des 
ROS Workspaces kopieren.

```
colcon build --packages-select kinect_unity_empfaenger
source install/setup.bash
ros2 run kinect_unity_empfaenger kin_empfaenger
``` 

Zum Visualisieren in ```rviz2``` die Datei [kinect1_rviz](kinect1_rviz.rviz)
