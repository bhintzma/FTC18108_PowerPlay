/* Copyright (c) 2022 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.DefineRobot.PowerPlayBot;
import static org.firstinspires.ftc.teamcode.DefineRobot.PowerPlayBot.DRIVE_SPEED;
import static org.firstinspires.ftc.teamcode.DefineRobot.PowerPlayBot.TURN_SPEED;

/**
 *  This file illustrates the concept of driving an autonomous path based on Gyro heading and encoder counts.
 *  The code is structured as a LinearOpMode
 *
 *  The path to be followed by the robot is built from a series of drive, turn or pause steps.
 *  Each step on the path is defined by a single function call, and these can be strung together in any order.
 *
 *  The code REQUIRES that you have encoders on the drive motors, otherwise you should use: RobotAutoDriveByTime;
 *
 *  This code ALSO requires that you have a BOSCH BNO055 IMU, otherwise you would use: RobotAutoDriveByEncoder;
 *  This IMU is found in REV Control/Expansion Hubs shipped prior to July 2022, and possibly also on later models.
 *  To run as written, the Control/Expansion hub should be mounted horizontally on a flat part of the robot chassis.
 *
 *  This sample requires that the drive Motors have been configured with names : left_drive and right_drive.
 *  It also requires that a positive power command moves both motors forward, and causes the encoders to count UP.
 *  So please verify that both of your motors move the robot forward on the first move.  If not, make the required correction.
 *  See the beginning of runOpMode() to set the FORWARD/REVERSE option for each motor.
 *
 *  This code uses RUN_TO_POSITION mode for driving straight, and RUN_USING_ENCODER mode for turning and holding.
 *  Note: You must call setTargetPosition() at least once before switching to RUN_TO_POSITION mode.
 *
 *  Notes:
 *
 *  All angles are referenced to the coordinate-frame that is set whenever resetHeading() is called.
 *  In this sample, the heading is reset when the Start button is touched on the Driver station.
 *  Note: It would be possible to reset the heading after each move, but this would accumulate steering errors.
 *
 *  The angle of movement/rotation is assumed to be a standardized rotation around the robot Z axis,
 *  which means that a Positive rotation is Counter Clockwise, looking down on the field.
 *  This is consistent with the FTC field coordinate conventions set out in the document:
 *  ftc_app\doc\tutorial\FTC_FieldCoordinateSystemDefinition.pdf
 *
 *  Control Approach.
 *
 *  To reach, or maintain a required heading, this code implements a basic Proportional Controller where:
 *
 *      Steering power = Heading Error * Proportional Gain.
 *
 *      "Heading Error" is calculated by taking the difference between the desired heading and the actual heading,
 *      and then "normalizing" it by converting it to a value in the +/- 180 degree range.
 *
 *      "Proportional Gain" is a constant that YOU choose to set the "strength" of the steering response.
 *
 *  Use Android Studio to Copy this Class, and Paste it into your "TeamCode" folder with a new name.
 *  Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name="Robot: Auto Drive By Gyro", group="Robot")
// @Disabled
public class AutoDriveByGyro_Linear extends LinearOpMode {

    @Override
    public void runOpMode() {

        // Make accessible the methods defined in PowerPlayBot
        // Pass the objects that are only defined once the OpMode starts running
        PowerPlayBot ppb = new PowerPlayBot(this, hardwareMap);

        // Initialize the drive system variables
        try {
            ppb.init();
        } catch (Exception e) {
            e.printStackTrace();
        }

        // Wait for the game to start (driver presses PLAY) Display IMU value while waiting
        while (opModeInInit()) {
            telemetry.addData("Robot Heading ", "= %4.0f", ppb.getRawHeading());
            telemetry.update();
        }

        // Play just pressed on Driver Station, reset Heading and game timer
        ppb.runtime.reset();

        // Step through each leg of the path,
        // Notes:   Reverse movement is obtained by setting a negative distance (not speed)
        //          holdHeading() is used after turns to let the heading stabilize
        //          Add a sleep(2000) after any step to keep the telemetry data visible for review

        ppb.driveStraight(DRIVE_SPEED, 24.0, 0.0);    // Drive Forward 24"
        ppb.turnToHeading(TURN_SPEED, -45.0);               // Turn  CW to -45 Degrees
        ppb.holdHeading(TURN_SPEED, -45.0, 0.5);   // Hold -45 Deg heading for a 1/2 second

        ppb.driveStraight(DRIVE_SPEED, 17.0, -45.0);  // Drive Forward 17" at -45 degrees (12"x and 12"y)
        ppb.turnToHeading(TURN_SPEED,  45.0);               // Turn  CCW  to  45 Degrees
        ppb.holdHeading(TURN_SPEED,  45.0, 0.5);    // Hold  45 Deg heading for a 1/2 second

        ppb.driveStraight(DRIVE_SPEED, 17.0, 45.0);  // Drive Forward 17" at 45 degrees (-12"x and 12"y)
        ppb.turnToHeading(TURN_SPEED,   0.0);               // Turn  CW  to 0 Degrees
        ppb.holdHeading(TURN_SPEED,   0.0, 1.0);    // Hold  0 Deg heading for 1 second

        ppb.driveStraight(DRIVE_SPEED,-48.0, 0.0);    // Drive in Reverse 48" (should return to approx. staring position)

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);  // Pause to display last telemetry message.
    }
}
