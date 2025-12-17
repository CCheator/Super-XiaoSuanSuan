Ubuntu调用Windows摄像头:

Ubuntu:
ifconfig # 查看 inet 地址 如 inet 192.168.209.70

Windows:
ffmpeg -list_devices true -f dshow -i dummy # 查看摄像头设备 如 "HP 5MP Camera"
ffmpeg -f dshow -i video="HP 5MP Camera" -c:v libx264 -preset ultrafast -tune zerolatency -g 30 -keyint_min 30 -sc_threshold 0 -x264-params repeat-headers=1 -f mpegts udp://192.168.209.70:5000 # 发送信号端口

sk-24b144b67ddf485aaf50335fbbe5c02e

conda activate xsuans
source install/setup.bash