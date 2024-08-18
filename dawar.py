import subprocess

def run_launch_file():
    package_name = "motors"
    launch_file = "RPI.launch"

    try:
        # Run the roslaunch command
        subprocess.run(["roslaunch", package_name, launch_file], check=True)
    except subprocess.CalledProcessError as e:
        print(f"Failed to run launch file: {e}")
    except Exception as e:
        print(f"An error occurred: {e}")

def run_another_script():
    script_path = "~/rpi_ws/stream.py"

    try:
        # Run the second Python script
        subprocess.run(["python", script_path], check=True)
    except subprocess.CalledProcessError as e:
        print(f"Failed to run the script: {e}")
    except Exception as e:
        print(f"An error occurred: {e}")

if __name__ == "__main__":
    # Run the ROS launch file
    run_launch_file()

    # Run the other Python script
    #run_another_script()
