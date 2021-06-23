var1=$(pgrep -x "python3")
var2=$(pgrep -x "roslaunch")
var3=$(pgrep -x "CarlaUE4.sh")
var4=$(pgrep -x "CarlaUE4-Linux-")
var5=$(pgrep -x "dbus-launch")

var="$var1 $var2 $var3 $var4 $var5"

for i in $var;
do
    echo $i
    kill -9 ${i}; 
done