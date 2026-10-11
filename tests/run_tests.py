import os
import subprocess
import sys

def main():
    os.environ["PYTEST_DISABLE_PLUGIN_AUTOLOAD"] = "1"
    subprocess.run([sys.executable, "-m", "pytest", "-vvv", "-rP"], check=True)

if __name__ == "__main__":
    sys.exit(main())
