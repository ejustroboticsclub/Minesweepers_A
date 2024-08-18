import subprocess
import os

def run_launch_file():
    package_name = "joy"
    launch_file = "PC.launch"

    try:
        # Run the roslaunch command
        subprocess.run(["roslaunch", package_name, launch_file], check=True)
    except subprocess.CalledProcessError as e:
        print(f"Failed to run launch file: {e}")
    except Exception as e:
        print(f"An error occurred while running the launch file: {e}")

def run_CovarianceGUI_script():
    script_path = os.path.expanduser("~/catkin_ws/src/robot_ekf/src/gui.py")
    try:
        # Run the Python script
        subprocess.run(["python3", script_path], check=True)
    except subprocess.CalledProcessError as e:
        print(f"Failed to run Covariance GUI script: {e}")
    except Exception as e:
        print(f"An error occurred while running the Covariance GUI script: {e}")

def run_streaming_script():
    script_path = os.path.expanduser("~/catkin_ws/src/YOLO/src/streaming.py")
    try:
        # Run the Python script
        subprocess.run(["python3", script_path], check=True)
    except subprocess.CalledProcessError as e:
        print(f"Failed to run streaming script: {e}")
    except Exception as e:
        print(f"An error occurred while running the streaming script: {e}")

if __name__ == "__main__":
    # Run the ROS launch file
    run_launch_file()

    # Run the Covariance GUI script
    run_CovarianceGUI_script()

    # Run the streaming script
    run_streaming_script()
