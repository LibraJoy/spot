# Spot
## /.bashrc setting
```
-export BOSDYN_CLIENT_USERNAME=user
-export BOSDYN_CLIENT_PASSWORD=scgau6g5w987
```
## Connnect to spot
```
cd ~/spot/src
python estop_gui.py
```
Ctrl + Z
```
bg
```
## Run spot navigation
-publish spot location
```
rosrun spot publisher.py
```
-subscribe waypoints and command spot
```
rosrun spot subscriber.py
```
