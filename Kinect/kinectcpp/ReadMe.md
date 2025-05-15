### Zu ergänzen
Es fehlt im kinect Ordner noch /include/kinect


### Starten des Pakets über 

''' 
colcon build
souce install/setup.bash
ros2 launch kinect multi_kinect.launch.py
'''

Falls colcon build nur unter --merge funktioniert folgendes durchführen: 

'''
mkdir -p ~/.colcon
nano ~/.colcon/defaults.yaml
'''

Dann 
'''
build:
merge-install: true
'''

Speichern & schließen (Strg+O, Enter, Strg+X)
