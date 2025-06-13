# AV Technology (Korea University Sejong Campus 2025)
Sinan KARAKAYA
주희
오지원
주희

## Description

This class had 2 distinct tasks, one for the midterms, the other for the finals. This repository contains both tasks.

### The midterms
Our task was to make the robot drive while staying between 2 yellow lanes. It also had the task of avoiding obstacles on the way.

The midterms are self contained in the `midterms.py` file.

#### How to run

To run the midterms:

```bash
source install/setup.bash
./startup_finals.sh # This was the same file between the finals and midterms
ros2 run av_technology midterms
```

### The finals

For the finals, the robot had to drive on roads, and follow driving rules such as traffic lights. It had to follow a certain path, and park in the designated parking spot thanks to a QR code.

The finals are contained in:
- `finals.py`: This is the main file
- `traffic_light_detector.py`
- `qrcode_detector.py`

All the other files are tests or other smaller projects that we had.

#### How to run

To run the finals, you need an extra repository that is unfortunately not included here.

You can find similar repositories online, and the overall steps would look like this:

```bash
# Into your repository that contains those tools
ros2 launch wego cartographer_launch.py
# Drive around to get scan the area
ros2 run nav2_map_server map_saver_cli -f map # This will save the scan as a .pgm file
```

Once you get your map, you can find the exact pose that represents your nodes into your graph (we represent our terrain as a graph, with different nodes and links. Each node represent a waypoint for our robot).

Then, to make it drive around:

```bash
# Into your repository that contains those tools
ros2 launch wego teleop_launch.py # This is only to launch the specific topcis needed, we don't actually use the features of teleop
ros2 launch wego navigation_diff_launch.py

# Into this repository
ros2 run av_technology traffic_light_detector
ros2 run av_technology qrcode_detector
ros2 run av_technology finals
```

It is important to note that our robot has special configuration files for the navigation_diff_launch node, which we cannot share here. To get satisfying result, we recommend decreasing the inflation and the robot surface, and increasing the tolerance in xy position and yaw rotation.