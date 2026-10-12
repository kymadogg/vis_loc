# Test Coverage
Current stats using `uv run test`:

```
Name                      Stmts   Miss Branch BrPart  Cover   Missing
---------------------------------------------------------------------
vis_loc/camera.py            63     63     24      0     0%   1-87
vis_loc/field.py             60      0     20      1    99%   92->85
vis_loc/localize.py         137     20     30      3    85%   50-60, 77, 299-300, 308-320
vis_loc/motion_model.py      56     23      2      0    57%   16-50
vis_loc/rendering.py         75     75     14      0     0%   1-142
vis_loc/robot.py             57     57     10      0     0%   1-79
vis_loc/utilities.py         49     11     24      2    79%   5-6, 9-10, 16, 26-31, 76
---------------------------------------------------------------------
TOTAL                       497    249    124      6    50%
```

## Camera
- 1-87: the entire thing

## Field
<!-- - 24-59: `plot_base()` -->
- 95->85: weird edge case where an occluded goal is found **first** then a non-occluded one is found

## Localize
- 50-60: `_resample()`
- 76-85: `_add_noise()`
<!-- - 109-111: if `self.debug == True` in `initial_draw()` -->
<!-- - 137: if `self.debug == True` in `estimated_pose()` -->
- 188: division by 0 case in `calc_error()`
<!-- - 230: if `total > 0` in `weight_particles()` -->
<!-- - 271: if the change in odometry (`delta_odom`) is none or 0 in `update()` -->
<!-- - 276-292: if `self.debug == True` in `update()` -->
- 299-300: if there are no weights available (`len(self.weights) == 0`)
- 308-320: resampling behavior in `update()`

## Motion Model
- 16-50: all of `differential()`
- 55-87 all of `omnidirectional()`

## Rendering
- 1-137: the entire thing

## Robot
- 1-80: the entire thing

## Utilities
- 6-7: all of `euclidean_distance()`
- 11-12: all of `percent_error()`
- 19: if `wrap_angle()` is given `deg=False`
- 27: if `ang_diff()` is given `deg=False`
- 31-36: all of `average_covariance()`
- 68: if `if t > u2` in `liang_barsky()`
- 82: returning `False, none` (not 100% sure what case of this gets you there)

