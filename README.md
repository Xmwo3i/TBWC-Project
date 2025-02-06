# Instructions for Setting up The Arduino Project:

- Download the Arduino IDE from https://www.arduino.cc/en/software if you haven't already.

- Download GitHub Desktop from https://desktop.github.com/ if you havent already and make an account.

- Now, you can either clone this repository through GitHub Desktop by clicking on the green 'code' button on the web view of the repo, or you can directly download it to your computer.

- Once you have this repository cloned or downloaded copy the *'Librares'* folder **into** the *Arduino* folder located in your documents on Windows, if it asks to replace files click yes. Note: when copying the Libraries folder you will copy **the actual folder and not the contents**.

- Finally you can open up the Arduino IDE and select the .ino file in the repo.

- Make sure you select the COM port of the arduino in the IDE and to upload and verify the code you can press the two buttons on the top left.

# How it Works

This arduino project utilizes a custom written PID Library as well as a custom kinematics model for differential drivetrain types.

The core of the functionality is in loop\(), where we read the linear acceleration and angular velocity from an MPU 6050 IMU. 

The linear acceleration is converted to linear velocity then is passed to a PID controller that calculates the optimal wheel speed. Next, angular velocity is passed to a different PID controller that calculates the optimal angular velocity to keep the gyro heading at 0 degrees. 

Finally the PID output of the linear velocity PID controller and the angular velocity PID controller is passed the kinematics model to calculate the left and right wheelspeeds that correlate to the velocities.
