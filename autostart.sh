PX4_DIR=~/PX4-Autopilot
AUTOSTART=4001
MODEL=gz_x500
PORT=8888

if [ ! -d "$PX4_DIR" ]; then
  echo "❌ PX4 目录不存在：$PX4_DIR"
  exit 1
fi

echo "✅ Starting PX4 instance..."

# 第一个（leader 机体, instance 0） export HEADLESS=1
gnome-terminal --tab -- bash -c "
export HEADLESS=1
export PX4_HOME_LAT=48.614056;
export PX4_HOME_LON=2.424111;
export PX4_HOME_ALT=87.47;
cd $PX4_DIR;
echo "Leader";
PX4_SYS_AUTOSTART=$AUTOSTART PX4_SIM_MODEL=$MODEL \
./build/px4_sitl_default/bin/px4 -i 0;
exec bash
"
sleep 7


# n个 follower
for i in 1 2 3 4; do
  idx=$((i-1))
  gnome-terminal --tab -- bash -c  "
cd $PX4_DIR;
echo "Follower" $i;
PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART=$AUTOSTART \
PX4_SIM_MODEL=$MODEL PX4_GZ_MODEL_POSE="0,$((5*i)),0,0,0,0" \
./build/px4_sitl_default/bin/px4 -i $i;
exec bash
"
sleep 3s
done

sleep 3s

echo "✅ 启动 MicroXRCEAgent..."
gnome-terminal --tab -- bash -c "
MicroXRCEAgent udp4 -p $PORT;
exec bash
"

