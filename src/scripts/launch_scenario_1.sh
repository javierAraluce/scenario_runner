echo 'Clean'
./scripts/Clean.sh

echo 'Bridge'

STATUS=$(/usr/bin/pgrep roslaunch | wc -l)
while [ $STATUS -eq 0 ]; do
    echo $STATUS
    roslaunch carla_ros_bridge carla_ros_bridge.launch &
    sleep 10
    STATUS=$(/usr/bin/pgrep roslaunch | wc -l)
done

echo 'Scenario'
sleep 2
python3 scenario_runner.py --route data_leaderboard/routes_devtest.xml data_leaderboard/all_towns_traffic_scenarios_public.json 0  --reloadWorld --sync --debug &
sleep 6 
python3 manual_control_explain.py 