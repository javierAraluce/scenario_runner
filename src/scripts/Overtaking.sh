echo 'Clean'
./scripts/Clean.sh


echo 'Mundo'
# cd /opt/carla-simulator/ && ./CarlaUE4.sh -ResX=300 -ResY=300 &
# cd ~/Tesis/carla/ && ./CarlaUE4.sh -ResX=300 -ResY=300 &
# sleep 10
echo 'Bridge'

STATUS=$(/usr/bin/pgrep roslaunch | wc -l)
while [ $STATUS -eq 0 ]; do
    echo $STATUS
    roslaunch carla_ros_bridge carla_ros_bridge.launch &
    sleep 10
    STATUS=$(/usr/bin/pgrep roslaunch | wc -l)
done

cd ~/Tesis/Repos/scenario_runner/src
echo 'Scenario'
python3 scenario_runner.py --openscenario t4ac_scenarios/t4ac_Overtaking_CCRM.xosc --sync &
sleep 6 
python3 manual_control_full_sensors.py -a -t 4.5 --res 5760x1080

echo 'Clean'
./scripts/Clean.sh