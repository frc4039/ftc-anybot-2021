# Instructions to Run this Code on a Robot
0. If you have not already, download AnybotTeleop.java file to your computer.

1. Connect to the Control Hub's WiFi Network. If you have not yet configured
   the WiFi, follow the "Getting Started with the Control Hub" guide:
   https://docs.revrobotics.com/control-hub/control-hub-gs

2. Open a web browser and navigate to
   http://192.168.43.1:8080/?page=java/editor.html

3. Click the "Upload Files" icon on the left, and select the AnybotTeleop.java
   file you downloaded earlier. The file will appear on the left. Click on it
   to open it. Then click on the wrench on the right to build the code. You
   should see "Build SUCCESSFUL!" appear in the bottom pane.

4. A robot configuration will need to be created in the app to work with
   this code. This is the motor configuration we used, but you may need to
   change it if you wired any motors to different ports.

   4.0) In the driver station app, tap on the three dots in the upper left,
        then Configure Robot.

   4.1) Tap "New" to create a new configuration, or click "Edit" on an
        existing configuration.

   4.2) Configure the robot as follows:
     * Control Hub Portal > Control Hub > Motors > Port 0\
       Type: Tetrix Motor                Name: rightDrive
     * Control Hub Portal > Control Hub > Motors > Port 1\
       Type: Tetrix Motor                Name: leftDrive
     * Control Hub Portal > Control Hub > Motors > Port 2\
       Type: Tetrix Motor                Name: elbow
     * Control Hub Portal > Control Hub > Motors > Port 3\
       Type: Tetrix Motor                Name: duckSpinner

     * Control Hub Portal > Control Hub > Servos > Port 0\
       Type: Continuous Rotation Servo   Name: intake\
       _This servo must be set to continuous rotation mode by the Smart Robot Servo Programmer included in the kit._

   4.3) Click "Done" to get to the top menu and then "Save". Enter a name
        and click "OK".

   4.4) Click the "Activate" button under the new configuration. Then use
        the Android back button to return to the app. The robot will reboot.

5. Connect the controller, and press Start + A. A controller icon should
   appear in the upper right corner of the app.

6. Click the arrow on the right to select "Anybot Teleop". When you are
   ready to run the code, press the "INIT" button, then the play button.
   A stop button will be in the same position as the play button was.
