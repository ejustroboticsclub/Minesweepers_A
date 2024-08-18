import tkinter as tk

class CovarianceGUI:
    def __init__(self, master):
        self.master = master
        self.master.title("Covariance GUI")

        # Create sliders with larger length
        self.odom_covariance_sliders = [
            self.create_slider(master, "odom_covariance_1", 0.0),
            self.create_slider(master, "odom_covariance_2", 0.0),
            self.create_slider(master, "odom_covariance_3", 0.0),
            self.create_slider(master, "odom_covariance_4", 0.0)
        ]

        self.imu_covariance_sliders = [
            self.create_slider(master, "imu_covariance_1", 0.0),
            self.create_slider(master, "imu_covariance_2", 0.0)
        ]

        # Add a button to update values
        self.update_button = tk.Button(master, text="Update Values", command=self.update_values)
        self.update_button.pack(pady=10)

    def create_slider(self, master, label, initial_value):
        frame = tk.Frame(master)
        frame.pack(padx=10, pady=5)

        tk.Label(frame, text=label).pack(side=tk.LEFT)
        slider = tk.Scale(frame, from_=0.0, to=1.0, resolution=0.005, orient=tk.HORIZONTAL, length=1000)  # Adjust length here
        slider.set(initial_value)
        slider.pack(side=tk.LEFT)
        
        return slider

    def update_values(self):
        odom_covariance = [slider.get() for slider in self.odom_covariance_sliders]
        imu_covariance = [slider.get() for slider in self.imu_covariance_sliders]

        print("Updated odom_covariance:", odom_covariance)
        print("Updated imu_covariance:", imu_covariance)

if __name__ == "__main__":
    root = tk.Tk()
    gui = CovarianceGUI(root)
    root.mainloop()
