# Visual Localization: VEX Override
Localizing using only relative heading of certain objects proof of concept. Or really localizing with anything but a lidar because why not?

matplotlib notebook/visualizer: `main.ipynb`

![Jupyter Notebook animation](.docs/omni_model_testing.gif)

> Soon to be made into a ROS package.

## How the field is represented

```
colors_by_index = {
    0: 'black',      # center
    1: 'gray', 2: 'gray', 3: 'gray', 4: 'gray',  # neutral
    5: 'red', 6: 'red',      # red goals
    7: 'blue', 8: 'blue'     # blue goals
}
```