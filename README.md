# ACC_2027_Competition

<b>How to Run</b>

Navigate into the directory that contains main.py (ex: "cd Desktop/main"), and then run "PYTHONPATH=$PYTHONPATH python3 main.py".

The project separates configuration (`app_config.py`), hardware adapters (`hardware_adapters.py`), mission planning (`mission_planning.py` and `mission_runtime.py`), camera/model processing (`camera_runtime.py` and `perception.py`), and traffic-condition detection (`traffic_control.py`). The route planner, controllers, perception helpers, and extracted runtime sections can be tested with `python -m unittest discover -s tests -v`.

On Windows, install Python 3.12 with `winget install --id Python.Python.3.12 --exact`. Open a new terminal and check the installation with `py -3.12 --version`, then run the tests with `py -3.12 -m unittest discover -s tests -v`.

The controller tests use NumPy, which is already required by the QCar runtime. On a standalone Python installation, install it with `py -3.12 -m pip install numpy` before running the tests.

If you encounter the following error:

Unable to download model /home/nvidia/Documents/Quanser/0_libraries/resources/applications/QCarDepthAlign/QCar2DepthAlign.rt-linux_qcar2. An operating system function returned an unrecognized error.

Then run "sudo pkill -9 python"

If you encounter it again after running "sudo pkill -9 python", try to ctrl+c and exit the program, then re-run "PYTHONPATH=$PYTHONPATH python3 main.py".
