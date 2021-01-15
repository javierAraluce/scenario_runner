echo 'Scenario'
python3 scenario_runner.py --route data_leaderboard/routes_devtest.xml data_leaderboard/all_towns_traffic_scenarios_public.json 0 --reloadWorld --debug &
python3 manual_control_full_sensors.py