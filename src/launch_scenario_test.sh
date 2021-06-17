# echo 'Scenario'
# sleep 2
# python3 scenario_runner.py --openscenario t4ac_scenarios/t4ac_ACC_CCRM.xosc --sync &
# sleep 6 
# python3 manual_control_full_sensors.py -a -t 3 --res 5760x1080


echo 'Scenario'
sleep 2
python3 scenario_runner.py --openscenario t4ac_scenarios/t4ac_Overtaking_CCRM.xosc --sync &
sleep 6 
python3 manual_control_full_sensors.py -a -t 3 --res 5760x1080