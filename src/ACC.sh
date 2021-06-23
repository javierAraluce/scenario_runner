echo 'Mundo'
cd /opt/carla-simulator/ && ./CarlaUE4.sh -ResX=300 -ResY=300 &
sleep 10
echo 'Bridge'
roslaunch carla_ros_bridge carla_ros_bridge.launch &
sleep 10
cd ~/Tesis/Repos/scenario_runner/src
echo 'Scenario'
python3 scenario_runner.py --openscenario t4ac_scenarios/t4ac_ACC_CCRM.xosc --sync &
sleep 6 
python3 manual_control_full_sensors.py -a -t 3 --res 5760x1080