    /* Copyright (c) 2017 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode.TeleOp;

    import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
    import com.qualcomm.robotcore.util.Range;
    import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

    import org.firstinspires.ftc.teamcode.DefineRobot.PowerPlayBot;
    import static org.firstinspires.ftc.teamcode.DefineRobot.PowerPlayBot.DRIVE_SPEED;
    import static org.firstinspires.ftc.teamcode.DefineRobot.PowerPlayBot.TURN_SPEED;

    /**
     * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
     * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
     * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
     * class is instantiated on the Robot Controller and executed.
     *
     * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
     * It includes all the skeletal structure that all linear OpModes contain.
     *
     * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
     * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
     */

    @TeleOp(name="JunctionGridDriving")
    // @Disabled
    public class JunctionGridDriving extends LinearOpMode {

        // Define constants for TETRIX Drivetrain
        // static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
        // static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
        // static final double     WHEEL_DIAMETER_INCHES   = 2.953 ;     // For figuring circumference
        // static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
        //         (WHEEL_DIAMETER_INCHES * 3.1415);
        // static final double     DRIVE_SPEED             = 0.6;
        // static final double     TURN_SPEED              = 0.5;
        // static final int        COUNTS_PER_1_TILE_STANDARD  = 3500;
        // static final int        COUNTS_PER_1_TILE_STRAFE    = 4000;

        // Define constants for goBilda Drivetrain
        // TBD...

        // Setup variables used during driving loop
        // Drive wheel power to set motor speed and display telemetry
        // double leftPower = 0.0;
        // double rightPower = 0.0;
        // double drive1 = 0.0;
        // double drive2 = 0.0;
        // double turn1 = 0.0;
        // double turn2 = 0.0;
        boolean buttonGridDriving = false;
        boolean buttonGridDrivingForward = false;
        boolean joystickGridStrafing = false;
        boolean joystickGridTurning = false;
        // int startMotorCounts = 0;
        // int stopMotorCounts = 0;

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
            ppb.gamepadSetCurrentBotCoordinates();

            // Wait for the game to start (driver presses PLAY) Display IMU value while waiting
            while (opModeInInit()) {
                telemetry.addData("Robot Heading ", "= %4.0f", ppb.getRawHeading());
                telemetry.addData("Bot Coordinates", "X: %2d Y: %2d", ppb.currentBotCol, ppb.currentBotRow);
                telemetry.update();
            }

            // PLAY just pressed on Driver Station, reset Heading and game timer
            ppb.runtime.reset();

            // run until the end of the match (driver presses STOP)
            while (opModeIsActive()) {

                // If buttonGrid driving, decide if the bot has driven far enough to stop, ignore all gamepad buttons
                if (buttonGridDriving) {
                    if (buttonGridDrivingForward) {
                        if (ppb.frontLeft.getCurrentPosition() >= ppb.stopMotorCounts) {
                            buttonGridDriving = false;
                            buttonGridDrivingForward = false;
                            ppb.stop();
                            /*
                            ppb.frontLeft.setPower(0.0);
                            ppb.frontRight.setPower(0.0);
                            ppb.backLeft.setPower(0.0);
                            ppb.backRight.setPower(0.0);
                             */
                        }
                    } else {
                        if (ppb.frontLeft.getCurrentPosition() <= ppb.stopMotorCounts) {
                            buttonGridDriving = false;
                            buttonGridDrivingForward = false;
                            ppb.stop();
                            /*
                            ppb.frontLeft.setPower(0.0);
                            ppb.frontRight.setPower(0.0);
                            ppb.backLeft.setPower(0.0);
                            ppb.backRight.setPower(0.0);
                             */
                        }
                    }
                } else {
                    // If not buttonGrid driving, check if N/S/E/W buttons newly pressed, if so start buttonGrid driving

                    // Buttons for driving one field tile North / South / East / West
                    ppb.gp1DpadUpLastState = ppb.gp1DpadUpCurrentState;
                    ppb.gp1DpadUpCurrentState = gamepad1.dpad_up;
                    ppb.gp1DpadRightLastState = ppb.gp1DpadRightCurrentState;
                    ppb.gp1DpadRightCurrentState = gamepad1.dpad_right;
                    ppb.gp1DpadDownLastState = ppb.gp1DpadDownCurrentState;
                    ppb.gp1DpadDownCurrentState = gamepad1.dpad_down;
                    ppb.gp1DpadLeftLastState = ppb.gp1DpadLeftCurrentState;
                    ppb.gp1DpadLeftCurrentState = gamepad1.dpad_left;

                    // Drive North by pressing dpad_up, if button state different from last loop then button is newly pressed
                    if (ppb.gp1DpadUpCurrentState && !ppb.gp1DpadUpLastState) {
                        buttonGridDriving = true;
                        buttonGridDrivingForward = true;
                        ppb.startMotorCounts = ppb.frontLeft.getCurrentPosition();
                        ppb.stopMotorCounts = ppb.startMotorCounts + ppb.COUNTS_PER_1_TILE_STANDARD;
                        // ppb.driveStraight(DRIVE_SPEED, 24.0, 0.0);    // Drive Forward 24"
                        ppb.drive1 = 1.0;
                        ppb.drive2 = 0.0;
                        ppb.leftPower = Range.clip(ppb.drive1 + ppb.drive2, -1.0, 1.0);
                        ppb.rightPower = Range.clip(ppb.drive1 - ppb.drive2, -1.0, 1.0);
                        // Send calculated power to wheels
                        ppb.frontLeft.setPower(ppb.rightPower);
                        ppb.frontRight.setPower(ppb.leftPower);
                        ppb.backLeft.setPower(ppb.leftPower);
                        ppb.backRight.setPower(ppb.rightPower);
                    }
                    // Drive East by pressing dpad_right
                    if (ppb.gp1DpadRightCurrentState && !ppb.gp1DpadRightLastState) {
                        buttonGridDriving = true;
                        buttonGridDrivingForward = true;
                        // ppb.strafeStraight(DRIVE_SPEED,24.0, -90.0);    // Strafe Right 24"
                        ppb.startMotorCounts = ppb.frontLeft.getCurrentPosition();
                        ppb.stopMotorCounts = ppb.startMotorCounts + ppb.COUNTS_PER_1_TILE_STRAFE;
                        ppb.drive1 = 0.0;
                        ppb.drive2 = -1.0;
                        ppb.leftPower = Range.clip(ppb.drive1 + ppb.drive2, -1.0, 1.0);
                        ppb.rightPower = Range.clip(ppb.drive1 - ppb.drive2, -1.0, 1.0);
                        // Send calculated power to wheels
                        ppb.frontLeft.setPower(ppb.rightPower);
                        ppb.frontRight.setPower(ppb.leftPower);
                        ppb.backLeft.setPower(ppb.leftPower);
                        ppb.backRight.setPower(ppb.rightPower);
                    }
                    // Drive South by pressing dpad_down
                    if (ppb.gp1DpadDownCurrentState && !ppb.gp1DpadDownLastState) {
                        buttonGridDriving = true;
                        buttonGridDrivingForward = false;
                        // ppb.driveStraight(DRIVE_SPEED,-24.0, 0.0);    // Drive Backward 24"
                        ppb.startMotorCounts = ppb.frontLeft.getCurrentPosition();
                        ppb.stopMotorCounts = ppb.startMotorCounts - ppb.COUNTS_PER_1_TILE_STANDARD;
                        ppb.drive1 = -1.0;
                        ppb.drive2 = 0.0;
                        ppb.leftPower = Range.clip(ppb.drive1 + ppb.drive2, -1.0, 1.0);
                        ppb.rightPower = Range.clip(ppb.drive1 - ppb.drive2, -1.0, 1.0);
                        // Send calculated power to wheels
                        ppb.frontLeft.setPower(ppb.rightPower);
                        ppb.frontRight.setPower(ppb.leftPower);
                        ppb.backLeft.setPower(ppb.leftPower);
                        ppb.backRight.setPower(ppb.rightPower);
                    }
                    // Drive West by pressing dpad_left
                    if (ppb.gp1DpadLeftCurrentState && !ppb.gp1DpadLeftLastState) {
                        buttonGridDriving = true;
                        buttonGridDrivingForward = false;
                        // ppb.strafeStraight(DRIVE_SPEED,24.0, 90.0);    // Strafe Left 24"
                        ppb.startMotorCounts = ppb.frontLeft.getCurrentPosition();
                        ppb.stopMotorCounts = ppb.startMotorCounts - ppb.COUNTS_PER_1_TILE_STRAFE;
                        ppb.drive1 = 0.0;
                        ppb.drive2 = 1.0;
                        ppb.leftPower = Range.clip(ppb.drive1 + ppb.drive2, -1.0, 1.0);
                        ppb.rightPower = Range.clip(ppb.drive1 - ppb.drive2, -1.0, 1.0);
                        // Send calculated power to wheels
                        ppb.frontLeft.setPower(ppb.rightPower);
                        ppb.frontRight.setPower(ppb.leftPower);
                        ppb.backLeft.setPower(ppb.leftPower);
                        ppb.backRight.setPower(ppb.rightPower);
                    }
                }

                // If not buttonGrid driving, check if joystickGrid driving
                if (!buttonGridDriving) {

                    // POV Mode uses left stick to go forward, and right stick to turn.
                    // - This uses basic math to combine motions and is easier to drive straight.
                    ppb.drive1 = -gamepad1.left_stick_y;
                    ppb.drive2 = -gamepad1.left_stick_x;
                    // turn1 = -gamepad1.right_stick_x;
                    // turn2 = -gamepad1.right_stick_y;

                    // If left joystick is deflected at all, do mecanum strafe driving
                    if ((ppb.drive1 != 0.0) || (ppb.drive2 != 0.0)){
                        ppb.leftPower = Range.clip(ppb.drive1 + ppb.drive2, -1.0, 1.0);
                        ppb.rightPower = Range.clip(ppb.drive1 - ppb.drive2, -1.0, 1.0);

                        // Send calculated power to wheels
                        ppb.frontLeft.setPower(ppb.rightPower);
                        ppb.frontRight.setPower(ppb.leftPower);
                        ppb.backLeft.setPower(ppb.leftPower);
                        ppb.backRight.setPower(ppb.rightPower);
                    } else {
                        // Only if left joystick is not deflected, check right joystick for rotation driving
                        // drive1 = -gamepad1.right_stick_y;
                        ppb.drive2 = -gamepad1.right_stick_x;
                        ppb.leftPower = Range.clip(ppb.drive1 + ppb.drive2, -1.0, 1.0);
                        ppb.rightPower = Range.clip(ppb.drive1 - ppb.drive2, -1.0, 1.0);

                        // Send calculated power to wheels
                        ppb.frontLeft.setPower(ppb.rightPower);
                        ppb.frontRight.setPower(ppb.leftPower);
                        ppb.backLeft.setPower(ppb.rightPower);
                        ppb.backRight.setPower(ppb.leftPower);

                    }
                }

                // Show the elapsed game time and wheel power.
                telemetry.addData("Status:",  "Run Time: " + ppb.runtime.toString());
                telemetry.addData("Wheel Power:", "Left: %.2f Right: %.2f", ppb.leftPower, ppb.rightPower);
                telemetry.addData("Counts:", "Start:%5d Stop:%5d", ppb.startMotorCounts, ppb.stopMotorCounts);
                telemetry.addData("Pos:", "LF:%5d RF:%5d LB:%5d RB:%5d", ppb.frontLeft.getCurrentPosition(), ppb.frontRight.getCurrentPosition(), ppb.backLeft.getCurrentPosition(), ppb.backRight.getCurrentPosition());
                telemetry.update();
            }
        }
    }
