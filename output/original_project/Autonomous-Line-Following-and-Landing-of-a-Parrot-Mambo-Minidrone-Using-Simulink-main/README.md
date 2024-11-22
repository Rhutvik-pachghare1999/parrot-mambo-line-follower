# Autonomous Line Following and Landing of a Parrot Mambo Minidrone Using Simulink

![image](https://github.com/user-attachments/assets/5fece421-f107-4162-a656-0fe91202a07f)

## Project Overview
Unmanned Aerial Vehicles (UAVs) are widely used for surveillance, automation, and smart navigation. This project implements an autonomous line-following and landing system on the Parrot Mambo minidrone using MATLAB Simulink. It combines image processing, behavioral logic, and hardware integration in a fully onboard control architecture. The project leverages MATLAB and Simulink for image processing, thresholding, and drone control, making it accessible for researchers, students, and hobbyists interested in UAV-based automation.

## Objectives
- Take off autonomously.
- Detect and follow a colored path using its downward-facing camera.
- Adjust its yaw and position based on the location of the path.
- Land automatically when the target landing circle is reached.

## Key Components
### Image Processing (Simulink)
The image processing system performs the following:
- Converts the camera feed (Y1UY2V format) into RGB.
- Applies RGB-based color thresholding using a MATLAB Function block.
- Splits the binary mask into subregions to detect where the path is:
  - **Up-left** (line on left)
  - **Up-right** (line on right)
  - **Center** (line centered)
- If the line disappears from all three zones (left, right, and top), it triggers landing logic.

### Stateflow Control Logic
The decision-making logic is implemented using two charts:
- **Chart 1 – Main FSM:**
  - Manages flight behavior in four states:
    - `Hover`: Maintain height after takeoff.
    - `Cruise`: Move based on yaw and velocity.
    - `Rotation`: Turn in place if line is off-center.
    - `Land`: Descend when the line disappears (i.e., drone is over the landing zone).
  - Position updates:
    ```
    dx = vel * cos(yaw)
    dy = vel * sin(yaw)
    xout = xout + dx
    yout = yout + dy
    ```
- **Chart 2 – STEER:**
  - Reads image signals and sets yaw direction:
    - `-1` → Yaw Left
    - `0` → Go Straight
    - `1` → Yaw Right

## Tools & Technologies

- MATLAB Simulink (R2023a or later recommended)
- Simulink Support Package for Parrot Minidrones
- Stateflow for behavior modeling
- Image Processing Toolbox

## How to Run This Project from Scratch
Follow these steps to get the project working on your drone:
### 1. **Install Required Toolboxes**
- Open MATLAB.
- Install:
  - Simulink Support Package for Parrot Minidrones
### 2. **Pair Your Drone**
- Turn on the Parrot Mambo and enable Bluetooth.
- Pair it with your computer via system Bluetooth settings.
### 3. **Open the Project (.prj) file**
- Once the Simulink file is open *right-click* on **flightControlSystem** block and select **Open as Top Model**
- Make sure the hardware settings target the "Parrot Mambo" board.
### 4. **Customize Parameters**
- You can adjust:
  - Velocity constants
  - RGB thresholds for color detection
  - Submatrix sizes for image region analysis
### 5. **Deploy the Code**
- Use **Build, Deploy & Start** from Simulink.
- The drone will:
  - Take off automatically
  - Follow the line path
  - Land when the target is centered
> **Note:** Ensure you test in a safe and open environment with consistent lighting for best performance.

## Results
- Line following and reactive to sharp turns.
- Yaw correction worked based on submatrix logic.
- The drone reliably detected the end-of-path condition and landed with precision.
- All logic ran onboard without PC intervention after deployment. <br>


![Untitled video - Made with Clipchamp (1)](https://github.com/user-attachments/assets/1193dfec-0a75-4560-a0ee-82f5e7155a09)


## Future Improvements

- Use adaptive thresholding for dynamic lighting environments.
- Add error recovery if the line is temporarily lost.
- Replace hard-coded values with tunable parameters for real-time tuning.
- Explore curved paths and intersections.

## Documentation and Video
- [Document](https://github.com/ChinmayAmrutkar/Autonomous-Line-Following-and-Landing-of-a-Parrot-Mambo-Minidrone-Using-Simulink/blob/main/Autonomous%20Line%20Following%20and%20Landing%20of%20a%20Parrot%20Mambo%20Minidrone%20Using%20Simulink.pdf)
- [Video](https://github.com/ChinmayAmrutkar/Autonomous-Line-Following-and-Landing-of-a-Parrot-Mambo-Minidrone-Using-Simulink/blob/main/video.mp4)

## 📄 License

This project is open-sourced for academic and educational use. See `LICENSE` for details.


## Acknowledgments

- [MathWorks Parrot Minidrone Examples](https://www.mathworks.com/help/supportpkg/parrot/)


_This repository is a hands-on project built from scratch to learn practical experience with real-time vision and model-based drone control._

