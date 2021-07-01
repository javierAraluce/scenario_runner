echo 'Clean'
./scripts/Clean.sh


echo 'Mundo'
cd /opt/carla-simulator/ && ./CarlaUE4.sh -ResX=300 -ResY=300 -port=2000 -carla-server -benchmark -fps=10 & 
sleep 10
echo 'Bridge'

# STATUS=$(/usr/bin/pgrep roslaunch | wc -l)
# while [ $STATUS -eq 0 ]; do
#     echo $STATUS
#     roslaunch carla_ros_bridge carla_ros_bridge.launch &
#     sleep 10
#     STATUS=$(/usr/bin/pgrep roslaunch | wc -l)
# done

cd ~/Tesis/Repos/scenario_runner/src
echo 'Scenario'
python3 scenario_runner.py --route data_leaderboard/routes_devtest.xml data_leaderboard/all_towns_traffic_scenarios_public.json 1  --sync --debug &
sleep 6 
python3 manual_control_testing.py -t 4.5 --res 5760x1080

echo 'Clean'
./scripts/Clean.sh