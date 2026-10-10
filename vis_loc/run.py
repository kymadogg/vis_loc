import sys
from IPython import start_ipython

def main():
    sys.argv = ["ipython", "main.ipynb"]
    start_ipython()
    
if __name__ == "__main__":
    sys.exit(main())
