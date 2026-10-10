# Visual Localization: VEX Override
[![CI](https://github.com/kymadogg/vis_loc/actions/workflows/CI.yml/badge.svg)](https://github.com/kymadogg/vis_loc/actions/workflows/CI.yml) <img alt="Static Badge" src="https://img.shields.io/badge/built%20with-uv-purple?style=flat&logo=astral">
<img alt="Static Badge" src="https://img.shields.io/badge/jupyter-ipynb-orange?style=flat&logo=jupyter">


Localizing using only relative heading of certain objects proof of concept. Or really localizing with anything but a lidar because why not?


Visualized using Matplotlib and Jupyter Notebook (`main.ipynb`)</br>
![Jupyter Notebook animation](.docs/omni_model_testing.gif) 
> Soon to be made into a ROS package.

## Running the Demo
```bash
# install uv
curl -LsSf https://astral.sh/uv/install.sh | sh

# sync uv
uv sync

# run the notebook
uv run sim
```

## Files, Classes, and Functions
Some short descriptions of the various things that can be found in this project.
### `field.py`

### `localize.py`

### `robot.py`

## Tests
There are some beginner tests in the `/tests` directory to help verify that the filter is working as it should.
```bash
# running all tests with pytest-cov
uv run test
```

## Things to Fix:
- [ ] how N = 0 is handled in `Localize.calc_error()`