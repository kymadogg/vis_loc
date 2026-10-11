# To-Do List
A general overview of what I need to get done, organized per type of fix (code, tests, actions, etc.)


## Code / Classes
Things that need to be fixed / tweaked in the core package code.
### Localize
- restructure particles variable to include weights
    - opt 1: make a dataclass / struct
    - opt 2: add a float element to the NDArray
- how N = 0 is handled in `Localize.calc_error()`
- add specific types to each instance attribute NDArray (i.e dtype=np.float32)

## Tests
things I need to fix and/or update in the tests
