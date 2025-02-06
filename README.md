# **Teddy Bear Wheel Chair (TBWC) Project**  

## **Table of Contents**  
1. [Overview](#overview)  
2. [Project Goals](#project-goals)  
3. [Competition Events](#competition-events)  
4. [Technical Details](#technical-details)  
   - [Hardware Components](#hardware-components)  
   - [Software Components](#software-components)  
5. [Project Workflow](#project-workflow)  
6. [Setup Instructions](#setup-instructions)  
7. [Software Implementation Instructions](#software-implementation-instructions)  
8. [Troubleshooting](#troubleshooting)  

## **Overview**  
The **Teddy Bear Wheel Chair (TBWC)** is a semester-long engineering project where student teams design, build, and test an Arduino-powered wheelchair for a teddy bear. The project is evaluated based on performance in a competition, design aesthetics, and engineering documentation.  

## **Project Goals**  
- Design and construct a functional TBWC.  
- Compete in events testing speed, agility, and accuracy.  
- Optimize the design for **low mass** and **aesthetic appeal**.  
- Document engineering analysis, performance, and improvements.  

## **Competition Events**  
Each year, the competition consists of two events. Past events include:  
- **Sprint Race** – Navigate a track as quickly as possible.  
- **Curling/Shooting** – Travel over a ramp and either stop at a target or shoot a ball into a goal.  

Performance is evaluated based on event results, weight minimization, and aesthetics.  

## **Technical Details**  

### **Hardware Components**  
- **Arduino Mega 2560** – Main microcontroller.  
- **MPU6050** – 6-axis motion sensor for stability and motion tracking.  
- **DC Motor & Motor Controller** – Provides movement control.  
- **Meccano Super Set** – Used to construct the wheelchair frame.  
- **Breadboard, Capacitors, Resistors** – For circuit assembly.  

Students supply their own batteries and may use additional materials within constraints.  

### **Software Components**  
The system utilizes various modules to ensure smooth operation:  
- **PID Controller** (`PIDController.cpp/h`) – Implements Proportional-Integral-Derivative (PID) control for precise motion regulation.  
- **Differential Kinematics Model** (`DifferentialKinematicsModel.cpp/h`) – Converts linear and angular velocity to individual wheel speeds.  
- **MPU6050 Motion Processing** (`MPU6050_6Axis_MotionApps612.cpp/h`) – Handles real-time motion data and calibration.  
- **Offset Calibration** (`OffsetConfig.cpp/h`) – Ensures sensor accuracy through calibration.  

## **Project Workflow**  
1. **Design Phase** – Plan wheelchair structure, electrical system, and control logic.  
2. **Development Phase** – Assemble components, implement motor control, and integrate sensors.  
3. **Testing & Iteration** – Adjust control parameters and refine stability.  
4. **Competition & Evaluation** – Compete and assess performance.  
5. **Final Report Submission** – Document design, performance analysis, and proposed improvements.  

## **Setup Instructions**  
1. **Hardware Assembly**  
   - Connect the **DC motor** to the motor controller and Arduino.  
   - Mount the **MPU6050 sensor** to the wheelchair frame.  
   - Construct the frame using **Meccano parts**.  
2. **Software Setup**  
   - Install the required Arduino libraries.  
   - Upload the provided **Arduino sketch** to the Mega 2560.  
   - Calibrate the **MPU6050 sensor** using the `OffsetConfig` module.  
3. **Testing & Debugging**  
   - Verify motor response with basic commands.  
   - Tune the **PID controller** for optimal movement.  
   - Perform trial runs and make adjustments.  

## **Software Implementation Instructions**  
1. **Install Required Libraries**  
   - Ensure you have the Arduino IDE installed.  
   - Install the `MPU6050` and `I2Cdev` libraries.  
   - Include necessary headers (`MPU6050.h`, `PIDController.h`, `DifferentialKinematicsModel.h`).  

2. **Configure the Microcontroller**  
   - Initialize the `MPU6050` sensor and configure offsets.  
   - Set up the `PIDController` with appropriate tuning values.  
   - Define the wheel movement using the `DifferentialKinematicsModel`.  

3. **Develop Control Logic**  
   - Implement a loop to read sensor data and update movement.  
   - Use the PID controller to regulate wheel speeds.  
   - Send motor control signals based on sensor feedback.  

4. **Upload and Test**  
   - Compile the code and upload it to the Arduino Mega 2560.  
   - Use serial monitoring to verify sensor data and PID adjustments.  
   - Conduct test runs and fine-tune parameters as needed.  

## **Troubleshooting**  
- **Motors not responding?** Check wiring connections and ensure the motor driver is powered.  
- **Unstable movement?** Recalibrate the MPU6050 sensor and adjust PID parameters.  
- **Wheelchair veering off track?** Check for unbalanced weight distribution and adjust motor speeds.  

---

Would you like any additional diagrams or code setup instructions?


