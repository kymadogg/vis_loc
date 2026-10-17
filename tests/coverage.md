# Test Coverage
Current stats using `uv run test`:

```
Name                      Stmts   Miss Branch BrPart  Cover   Missing
---------------------------------------------------------------------
vis_loc/camera.py            63     56     24      0     8%   8-14, 17-37, 42-87
vis_loc/field.py             60      0     20      1    99%   92->85
vis_loc/localize.py         137     20     30      3    85%   50-60, 77, 299-300, 308-320
vis_loc/motion_model.py      57      0      2      0   100%
vis_loc/rendering.py         74     63     14      0    12%   9-18, 30-35, 41-44, 57-79, 91-103, 116-146
vis_loc/robot.py             46     37      6      0    17%   9-17, 26-40, 59-82
vis_loc/utilities.py         49     11     24      2    79%   5-6, 9-10, 16, 26-31, 76
---------------------------------------------------------------------
TOTAL                       486    187    120      6    60%
```

## Camera
- 1-87: the entire thing

## Field
- 95->85: weird edge case where an occluded goal is found **first** then a non-occluded one is found

## Localize
- 50-60: `_resample()`
- 77: `_add_noise()`
- 299-300: if there are no weights available (`len(self.weights) == 0`)
- 308-320: resampling behavior in `update()`

## Rendering
- 9-18: class definition + instance attributes
- 30-35: all of `remove_element()`
- 41-44: all of `render()`
- 57-79: all of `particles()`
- 91-103: all of `estimated_pose()`
- 116-146: all of `camera_view()`

## Robot
- 9-17: class definition + instance attributes
- 26-40: all of `draw_robot()`
- 59-82: all of `drive_path()`

## Utilities
- 5-6: all of `euclidean_distance()`
- 9-10: all of `percent_error()`
- 16: if `wrap_angle()` is given `deg=False`
- 26-31: all of `average_covariance()`
- 76: returning `False, none` (not 100% sure what case of this gets you there)

