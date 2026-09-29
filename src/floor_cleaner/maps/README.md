# maps/

Saved maps live here. This folder is empty until you create your first map.

Save a map while `slam.launch.py` is running:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/floor_cleaner_ws/src/floor_cleaner/maps/my_map
```

This creates two files:

| File          | What it is |
|---------------|------------|
| `my_map.pgm`  | The picture of the map. One pixel = one 5 cm x 5 cm cell. White = free, black = wall/obstacle, grey = unknown (never seen by the LiDAR). |
| `my_map.yaml` | The description Nav2 needs to use the picture: image file name, `resolution` (metres per pixel), `origin` (where the bottom-left pixel is in the map frame) and the thresholds that decide which grey values count as occupied/free. |

`navigation.launch.py` loads `~/floor_cleaner_ws/src/floor_cleaner/maps/my_map.yaml`
by default. To use another map:

```bash
ros2 launch floor_cleaner navigation.launch.py map:=$HOME/floor_cleaner_ws/src/floor_cleaner/maps/other_map.yaml
```

Always keep the `.yaml` and `.pgm` together in the same folder; the yaml refers to
the image by a relative file name.
