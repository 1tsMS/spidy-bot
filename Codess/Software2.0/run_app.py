"""Start the Spidy app. Works from anywhere: double-click, VS Code Run, or `python run_app.py`."""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)          # so `import spidy` and `import app` resolve from this folder
os.chdir(HERE)

from app.main import main

main()
