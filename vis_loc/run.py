import sys
from IPython import start_ipython

#uv run python -m IPython main.ipynb


def main():
    sys.argv = ["ipython", "main.ipynb"]
    start_ipython()
    

if __name__ == "__main__":
    sys.exit(main())
