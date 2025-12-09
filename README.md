# Leap Motion Gesture Recognition

A brief description of the project, focusing on **real-time hand gesture recognition** using Leap Motion sensors and the **Robot Operating System (ROS)** framework. The system processes 3D joint data from one or two Leap Motion devices to recognize various hand gestures.

---

## Getting Started

### Install Required Libraries
1.  Install **ROS** (Robot Operating System) from the official website: [ROS](https://www.ros.org/).
2.  Follow the instructions to install the necessary libraries for Leap Motion (already included in the project): [LeapC Python Bindings](https://github.com/ultraleap/leapc-python-bindings/tree/main).

---

## Running on Windows Subsystem Linux (WSL)

To make your Leap Motion device accessible within your WSL environment, you need to use `usbipd` to bind the USB device:

1.  Open **PowerShell as Administrator**.
2.  Run the following command to list connected USB devices:
    ```powershell
    usbipd list
    ```
3.  **Identify the USB device ID** corresponding to your Leap Motion sensor.
4.  Bind the USB device for use in WSL. **Replace `4-4` with the correct Bus ID** you identified in the previous step:
    ```powershell
    usbipd bind --busid 4-4
    ```

---

## Building and Running in Ubuntu

### Build the Executable
1.  Navigate to the **colcon workspace**:
    ```bash
    cd colcon_ws
    ```
2.  Build the project using `colcon`:
    ```bash
    colcon build --symlink-install
    ```
3.  Source the setup file to make the executables available in your current shell:
    ```bash
    . install/setup.bash
    ```

### Run the Application
To launch the core applications, use the following `ros2 run` commands:
> *Verify these commands are correct based on the latest package names.*

```bash
ros2 run leap_gesture_recognition leap_gesture_recognition
ros2 run leap_motion leap_motion
```

###  ROS Nodes and Data Visualization

The scripts containing the ROS nodes are located inside the **`ros_nodes`** folder.

| Script | Node Functionality | Topic to Echo |
| :--- | :--- | :--- |
| **`leap_motion_desktop`** / **`leap_motion_screen`** | Nodes for **streaming JSON hand data** from two different Leap Motion sensors (Desktop/Screen orientations). | `/sensors/leapDesk/json` <br> `/sensors/leapScreen/json` |
| **`leap_fusion`** | Contains the node that **fuses the streaming data** from both Leap sensors for more robust recognition. | `/sensors/leapFusion/json` |
| **`recognizer_leap`** | Contains the node with the **gesture classifier** logic. | `/sensors/leapGest/json` |

To **visualize the output** of any topic, run:
```bash
ros2 topic echo /sensors/leapDesk/json
ros2 topic echo /sensors/leapScreen/json
ros2 topic echo /sensors/leapFusion/json
ros2 topic echo /sensors/leapGest/json
```
## Tracked Hand Joint Data
The system tracks 3D positions for key hand joints, structured as follows:

- **Palm & Wrist:** Palm position, rotation (quaternion), and wrist position.
- **Fingers:** Each finger (thumb, index, middle, ring, pinky) has:
  - Metacarpal
  - Proximal
  - Intermediate
  - Distal
  
Each joint is represented by its `x`, `y`, and `z` coordinates.

### Example Naming Convention
- `left_palm_x`, `left_palm_y`, `left_palm_z`
- `left_thumb_proximal_x`, `left_thumb_proximal_y`, `left_thumb_proximal_z`
- `right_index_distal_x`, `right_index_distal_y`, `right_index_distal_z`

This structure applies to both hands (`left_` and `right_`).


## Recognizable gestures
List of recognized gestures: [FIVE, OK, THUMB, NON_GESTURE]

## Add gestures and Train the model
The necessary training assets and scripts are located inside the **`gesture_recognizer_creator`** folder.

Within this directory:
* **Trained Networks** are saved in the **`nets`** sub-folder.
* **Training Data** is located in the **`data`** sub-folder, which is further divided into **`train`** and **`test`** directories.

### Add New Users
To add new users, simply create a new folder within the **`data`** directory, using the next sequential user number as the folder name (e.g., `user4`). This folder must contain the recorded data as `.txt` files.

### Add New Gestures
To add new gestures, record the data and save the `.txt` files inside the relevant user's folder. **The file name must be the exact name of the gesture.**

### Train and Test
1.  Launch the **`gesture_recognizer_training`** script.
2.  Test the training results with the **`gesture_recon_leap_working`** script.

The trained networks will be saved in the **`nets`** sub-folder. The script saves a new classifier after each run, including **SVM**, **LDA**, and **Bayesian** models.