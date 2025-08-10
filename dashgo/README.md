# build serial

edit `src/serial/build/CMakeCache.txt`

edit it to
```makefile
//Path to a program.
PYTHON_EXECUTABLE:FILEPATH=/usr/bin/python3
```

cd `src/serial` and `make`

# build dashgo

```bash
git checkout slam_02
sudo sh 'src/dashgo/dashgo_bringup/startup/create_dashgo_udev.sh'
```



```bash
sudo apt install ros-noetic-turtlebot3 python-serial
```

https://github.com/EAIBOT/dashgo.git

```bash
cd dashgo
ros-create-workspace
```

# edit dashgo

``cp hub-usb.rules /etc/udev/rules.d/`

`src/dashgo/dashgo_bringup/startup/dashgo-start`

```bash
source /opt/ros/noetic/setup.bash
source dashgo/devel/setup.bash
```


`src/dashgo/dashgo_bringup/nodes/dashgo_driver.py`
```python
#!/usr/bin python
```

`ROS/dashgo/src/dashgo/dashgo_bringup/config/my_dashgo_params.yaml`

```yaml
port: /dev/ttyUSB0
```

maybe
```bash
sudo uesrmod -a -G dialout YourUserName
```
