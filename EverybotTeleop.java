/* Copyright (c) 2021, MakeShift Robotics. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of MakeShift Robotics nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

/**
 * This is the example code for the teleoperated mode of the everybot.
 *
 * Setup required to run this code on your robot:
 *
 * 0. If you have not already, download this file to your computer.
 *
 * 1. Connect to the Control Hub's WiFi Network. If you have not yet configured
 *    the WiFi, follow the "Getting Started with the Control Hub" guide:
 *    https://docs.revrobotics.com/control-hub/control-hub-gs
 *
 * 2. Open a web browser and navigate to
 *    http://192.168.43.1:8080/?page=java/editor.html
 *
 * 3. Click the "Upload Files" icon on the left, and select this file.
 *
 * 4. A robot configuration will need to be created in the app to work with
 *    this code. This is the motor configuration we used, but you may need to
 *    change it if you wired any motors to different ports.
 *
 *    4.0) In the driver station app, tap on the three dots in the upper left,
 *         then Configure Robot.
 *
 *    4.1) Tap "New" to create a new configuration, or click "Edit" on an
 *         existing configuration.
 *
 *    4.2) Configure the robot as follows:
 *      * Control Hub Portal > Control Hub > Motors > Port 0
 *        Type: Tetrix Motor                Name: rightDrive
 *      * Control Hub Portal > Control Hub > Motors > Port 1
 *        Type: Tetrix Motor                Name: leftDrive
 *      * Control Hub Portal > Control Hub > Motors > Port 2
 *        Type: Tetrix Motor                Name: elbow
 *      * Control Hub Portal > Control Hub > Motors > Port 3
 *        Type: Tetrix Motor                Name: duckSpinner
 *
 *      * Control Hub Portal > Control Hub > Servos > Port 0
 *        Type: Continuous Rotation Servo   Name: intake
 *      [^This servo must be set to continuous rotation mode by the Smart
 *        Robot Servo Programmer included in the kit.]
 *
 *    4.3) Click "Done" to get to the top menu and then "Save". Enter a name
 *         and click "OK".
 *
 *    4.4) Click the "Activate" button under the new configuration. Then use
 *         the Android back button to return to the app. The robot will reboot.
 *
 * 5. Connect the controller, and press Start + A. A controller icon should
 *    appear in the upper right corner of the app.
 *
 * 6. Click the arrow on the right to select "Everybot Teleop". When you are
 *    ready to run the code, press the "INIT" button, then the play button.
 *    A stop button will be in the same position as the play button was.
 */

@TeleOp(name="Everybot Teleop", group="Iterative Opmode")

public class EverybotTeleop extends OpMode
{
    // Left and right drive motors.
    private DcMotor leftDrive;
    private DcMotor rightDrive;

    // The elbow controls the angle of the main arm.
    private DcMotor elbow;

    // Elbow motor setpoints adjust these to change the arm height.
    private final int elbowTop = -425;
    private final int elbowMid = -300;
    private final int elbowBottom = -175;

    // This controls the elbow speed.
    private final double elbowPower = 0.2;

    // The duck spinner is the tower that spins the carousel.
    private DcMotor duckSpinner;

    // The intake servo is used to actually grip and release the freight.
    private CRServo intake;


    /** This code runs one time when the robot is initialized. */
    @Override
    public void init() {
        // Get the hardware variables. These must match the names and types
        // selected for the active configuration in the app.
        leftDrive  = hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");
        elbow = hardwareMap.get(DcMotor.class, "elbow");
        duckSpinner = hardwareMap.get(DcMotor.class, "duckSpinner");
        intake = hardwareMap.get(CRServo.class, "intake");

        // Turn off the elbow until a button is pressed.
        elbow.setPower(0);
        elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // If we send +1.0 power to each drive motor, the robot will turn. This
        // is because the one motor is rotated 180 degrees from the other. To
        // fix this, we need to invert the motor direction on one side.
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /** This code runs repeatedly while the robot is initialized, but not
     *  yet running.
     */
    @Override
    public void init_loop() {
        // Show the elbow position on the driver station app. This is used to
        // measure the arm setpoints.
        telemetry.addData("Elbow Position", elbow.getCurrentPosition());
        telemetry.update();
    }

    /** This code runs one time when the robot begins running. */
    @Override
    public void start() {
    }

    /** This code runs repeatedly while the robot is running. */
    @Override
    public void loop() {
        // We will use one stick for forward and back; the other for turning.
        double drive = gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;

        // Square the driver inputs. This can make it easier to control at low
        // speeds.
        drive = drive * drive * Math.signum(drive);
        turn = turn * turn * Math.signum(turn);

        leftDrive.setPower(Range.clip(drive + turn, -1.0, 1.0));
        rightDrive.setPower(Range.clip(drive - turn, -1.0, 1.0));

        // Intake will be run in one direction when the left trigger is
        // pressed, and in the other when the right trigger is. They are analog
        // axes, so pressing down harder will run the intake faster.
        double intakePower = gamepad1.left_trigger - gamepad1.right_trigger;
        intake.setPower(intakePower);

        // If either bumper was pressed run the spinner, otherwise stop it.
        boolean runIntake = gamepad1.left_bumper || gamepad1.right_bumper;
        double duckSpinnerPower = runIntake ? 1.0 : 0.0;
        duckSpinner.setPower(duckSpinnerPower);

        // Calculate elbow height if a button was pressed.
        if(gamepad1.y) {
            // Y = Top / Shipping Hub Level 3
            elbow.setTargetPosition(elbowTop);
            elbow.setPower(elbowPower);
            elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else if(gamepad1.b) {
            // B = Middle / Shipping Hub Level 2
            elbow.setTargetPosition(elbowMid);
            elbow.setPower(elbowPower);
            elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else if(gamepad1.a) {
            // A = Low / Shipping Hub Level 1
            elbow.setTargetPosition(elbowBottom);
            elbow.setPower(elbowPower);
            elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else if (gamepad1.x) {
            // X = Bottom / Pickup
            elbow.setPower(0);
            elbow.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        // Show current elbow position for debugging.
        telemetry.addData("Elbow Position", elbow.getCurrentPosition());
        telemetry.update();
    }

    /** This code runs one time when the robot stops. */
    @Override
    public void stop() {
        // Stop the arm motor when stopped.
        elbow.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        elbow.setPower(0);
    }
}
