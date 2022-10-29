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

import static org.firstinspires.ftc.teamcode.DefineRobot.PowerPlayBot.DRIVE_SPEED;
import static org.firstinspires.ftc.teamcode.DefineRobot.PowerPlayBot.TURN_SPEED;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.DefineRobot.PowerPlayBot;

/**
 Autonomous code for competition - PowerPlay season
 */

@Autonomous(name="LM0_Red_Right", group="Robot")
// @Disabled
public class LM0_Red_Right extends LinearOpMode {

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

        // Set initial Bot Coordinates on the field
        ppb.currentBotCol = ppb.RED_RIGHT_START_COL;
        ppb.currentBotRow = ppb.RED_RIGHT_START_ROW;
        ppb.currentHeading = 0.0;
        ppb.currentAlliance = PowerPlayBot.Alliance.RED;

        // Wait for the game to start (driver presses PLAY) Display IMU value while waiting
        while (opModeInInit()) {
            telemetry.addData("Current Alliance", "%s", ppb.currentAlliance);
            telemetry.addData("Robot Heading ", "= %4.0f", ppb.getRawHeading());
            telemetry.addData("Start Position",  "X: %2d Y: %2d", ppb.currentBotCol, ppb.currentBotRow);
            telemetry.update();
        }

        // PLAY just pressed on Driver Station, reset Heading and game timer
        ppb.runtime.reset();

        /*
         Autonomous code goes here
        */

        // Move from starting position against the wall to the center of the current tile
        ppb.driveStraight(DRIVE_SPEED, 4.0, 0.0);    // Drive Forward 4"
        ppb.holdHeading(TURN_SPEED, 0.0, 0.25);      // Hold 0 Deg heading for a 0.25 second
    /*

        // Vision Detect for the Sleeve goes here

        // Drive to the closest stack of cones for our Alliance color
        // Drive to all the High Junctions + closest Medium Junction to deposit cones
        // Get Stack Cone #5
        ppb.openClaw();
        ppb.moveSlidesToHeight(ppb.STACK_CONE_5);
        ppb.driveToCoordinate(ppb.RED_FRONT_CONE_STACK_COL, ppb.RED_FRONT_CONE_STACK_ROW);
        ppb.closeClaw();
        // Drive to High Junction (5,6)
        ppb.driveToCoordinate(4, 6);
        ppb.moveSlidesToHeight(ppb.JUNCTION_HIGH);
        ppb.driveToCoordinate(5, 6);
        ppb.openClaw();
        // Get Stack Cone #4
        ppb.driveToCoordinate(4, 6);
        ppb.moveSlidesToHeight(ppb.STACK_CONE_4);
        ppb.driveToCoordinate(ppb.RED_FRONT_CONE_STACK_COL, ppb.RED_FRONT_CONE_STACK_ROW);
        ppb.closeClaw();
        // Drive to High Junction (7,4)
        ppb.driveToCoordinate(6, 4);
        ppb.moveSlidesToHeight(ppb.JUNCTION_HIGH);
        ppb.driveToCoordinate(7, 4);
        ppb.openClaw();
        // Get Stack Cone #3
        ppb.driveToCoordinate(6, 4);
        ppb.moveSlidesToHeight(ppb.STACK_CONE_3);
        ppb.driveToCoordinate(ppb.RED_FRONT_CONE_STACK_COL, ppb.RED_FRONT_CONE_STACK_ROW);
        ppb.closeClaw();
        // Drive to High Junction (3,4)
        ppb.driveToCoordinate(4, 4);
        ppb.moveSlidesToHeight(ppb.JUNCTION_HIGH);
        ppb.driveToCoordinate(3, 4);
        ppb.openClaw();
        // Get Stack Cone #2
        ppb.driveToCoordinate(4, 4);
        ppb.moveSlidesToHeight(ppb.STACK_CONE_2);
        ppb.driveToCoordinate(ppb.RED_FRONT_CONE_STACK_COL, ppb.RED_FRONT_CONE_STACK_ROW);
        ppb.closeClaw();
        // Drive to High Junction (5,2)
        ppb.driveToCoordinate(4, 2);
        ppb.moveSlidesToHeight(ppb.JUNCTION_HIGH);
        ppb.driveToCoordinate(5, 2);
        ppb.openClaw();
        // Get Stack Cone #1
        ppb.driveToCoordinate(4, 2);
        ppb.moveSlidesToHeight(ppb.STACK_CONE_1);
        ppb.driveToCoordinate(ppb.RED_FRONT_CONE_STACK_COL, ppb.RED_FRONT_CONE_STACK_ROW);
        ppb.closeClaw();
        // Drive to Medium Junction (2,2)
        ppb.driveToCoordinate(4, 2);
        ppb.moveSlidesToHeight(ppb.JUNCTION_HIGH);
        ppb.driveToCoordinate(3, 2);
        ppb.openClaw();
        // Park for end of Autonomous
        ppb.driveToCoordinate(2, 2);
        ppb.moveSlidesToHeight(ppb.JUNCTION_GROUND);
*/
        telemetry.addData("Current Alliance", "%s", ppb.currentAlliance);
        telemetry.addData("Current Position", "X: %2d Y: %2d", ppb.currentBotCol, ppb.currentBotRow);
        telemetry.addData("Red Right Path", "Complete");
        telemetry.update();
        sleep(1000);  // Pause to display last telemetry message.
    }
}
