# rosbag analysis

These scripts are used to extract human data from rosbags. First, make sure you run the human detection ros node before playing the rosbags:

```bash
cd ~/catkin_ws
source devel/setup.bash
roslaunch sp_perception leg.launch
```

Then, to process a single rosbag:

```bash
cd ./scripts
python process.py main --name {participant id} --bag {path to bag file} --speed {playback speed, default to 1} --start_time {when to start processing (in secs), defaults to 0}
```

To batch process, run:

```bash
cd ./scripts
python process.py batch --folder {folder for all files, organized by id} --start-from {which user to start processing, optional}
```
