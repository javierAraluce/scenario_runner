echo 'Scenario'
sleep 2
python3 scenario_runner.py --route data_leaderboard/routes_devtest.xml data_leaderboard/all_towns_traffic_scenarios_public.json 2 --reloadWorld --debug &
sleep 6 
python3 manual_control_full_sensors.py