# NextPal Windows 控制台

## 启动
双击bat文件打开程序
```powershell
py -m pip install -r requirements.txt
py bluetooth_touch.py
```

程序默认连接 `tcp://192.168.100.102:8554` 获取 2K300 的 USB 摄像头画面。
地址变化时，点击右上角“设置”，修改“视频地址”后点击“重新连接”。没有收到画面时，
界面自动显示 `robot_dog_default.jpg`，视频恢复后自动切回实时画面。

摄像头卡片支持自由调节：拖动卡片右侧的竖向分隔线调整左右宽度，拖动摄像头区域
下方的横向分隔线调整上下高度。

## Wi-Fi 图传

摄像头仍物理连接在 2K300 的 USB 口；电脑与 2K300 之间不需要数据线。两台设备连接
同一个 Wi-Fi，电脑能访问板卡 IP 的 TCP 8554 端口即可图传。若路由器 DHCP 改变了
板卡地址，需要在“视频地址”中填写新的地址，或在路由器中给 2K300 设置 DHCP 地址
保留。可在 Windows 上检查：

```powershell
Test-NetConnection 192.168.100.102 -Port 8554
```

板端视频通道集成在现有 `camera_fb` 进程中，不会再次打开 `/dev/video0`。板端服务仍使用：

```sh
systemctl status camera-view.service --no-pager
```
