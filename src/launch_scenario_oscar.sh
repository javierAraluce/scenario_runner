echo 'Scenario  Oscar Pc'
sleep 2
python3 scenario_runner.py --route data_leaderboard/routes_devtest.xml data_leaderboard/all_towns_traffic_scenarios_public.json 0 --reloadWorld --host 192.168.75.134 --debug &
sleep 6 
python3 manual_control_full_sensors.py --host 192.168.75.134