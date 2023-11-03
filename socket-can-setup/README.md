# socketcan-setup

A systemd service to bring up the SocketCAN `can0` channel automatically when the
USB to CAN adapter is plugged in.

Install:
```bash
sudo cp setup-socketcan.sh /usr/bin
sudo cp setup-socketcan.service /lib/systemd/system
sudo systemctl start setup-socketcan
sudo systemctl enable setup-socketcan
```

Check the status:
```bash
sudo journalctl -fu setup-socketcan
```