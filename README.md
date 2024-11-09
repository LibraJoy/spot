# Spot
## /.bashrc setting
add the following lines to ~/.bashrc
```
export BOSDYN_CLIENT_USERNAME=user
export BOSDYN_CLIENT_PASSWORD=scgau6g5w987
export SPOT_HOSTNAME="192.168.80.3"
```
## Install required dependencies and the package
```
mkdir -p spot_ws/src
cd spot_ws/src
git clone https://github.com/LibraJoy/spot.git
cd spot/src/spot
pip install -r requirements.txt
cd ~/spot_ws
catkin_make
```
## Connnect to spot
change the hostname in spot_spot and estop modules
- via ethernet: "192.168.50.3"
- via spot wifi: "192.168.80.3"

Always run estop first
```
cd ~/spot_ws/src/spot/src/spot
python estop_nogui.py
```

## Run spot
- publish all necessary info for mapping and exploration
```
rosrun spot spot_base.py
```
- publish spot CAM images
```
rosrun spot img_pub.py
```
- publish spot odom and pose
```
rosrun spot publisher.py
```
- subscribe waypoints and command spot
```
rosrun spot subscriber.py
```
