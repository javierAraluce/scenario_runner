var1=$(pgrep -x "python3")
var2=$(pgrep -x "roslaunch")
var3=$(pgrep -x "CarlaUE4.sh")
var4=$(pgrep -x "CarlaUE4-Linux-")
var5=$(pgrep -x "dbus-launch")
var6=$(pgrep -x "python3 <defunct>")
var7=$(pgrep -x "python")
var8=$(pgrep -x "rosmaster")
var9=$(pgrep -x "dbus-daemon")
var10=$(pgrep -x "rosout")

var="$var1 $var2 $var3 $var4 $var5 $var6 $var7 $var8 $var9 $var10"

for i in $var;
do
    echo $i
    kill -9 ${i}; 
done