# rosbag_to_csv

ROS2 node that subscribes to a set of topics and logs incoming messages to CSV files (one file per topic).

## Build

```bash
cd ~/rosbag_to_csv
colcon build --packages-select csv_logger
source install/setup.bash
```

## Run

1. Start playback of your bag (in one terminal):
   ```bash
   ros2 bag play <path_to_bag>
   ```
2. In another terminal (after sourcing `install/setup.bash`), run the logger:
   ```bash
   ros2 run csv_logger csv_logger_node
   ```

CSV files are written to `/home/zweminhtetaung/CrazySim/data/data_ballistic_planner/hop_tracking/<timestamp>/`, one CSV per topic.

Edit `TOPICS_TO_SUBSCRIBE` in [csv_logger_node.py](src/csv_logger/csv_logger/csv_logger_node.py) to change which topics get logged.
