/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.Autonomous.AprilTagVision;

import static org.firstinspires.ftc.teamcode.DefineRobot.PowerPlayBot.AMAX_POS;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.DefineRobot.PowerPlayBot;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

/* Test for LM1 - Nov 10 2022 */

@Autonomous
public class LM1_BlueRight_RedLeft extends LinearOpMode
{

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    // Tag IDs from the 36h11 family
    int LEFT = 0;
    int MIDDLE = 1;
    int RIGHT = 2;

    // Slide Encoder Heights
    int     JUNCTION_HIGH             = 2885;    // Height of junctions - highest
    int     JUNCTION_MEDIUM           = 2150;    // Height of junctions - medium
    int     JUNCTION_LOW              = 1400;     // Height of junctions - shortest
    int     JUNCTION_GROUND           = 100;

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode()
    {

        PowerPlayBot ppb = new PowerPlayBot(this, hardwareMap);

        // Initialize the drive system variables
        try {
            ppb.init();
        } catch (Exception e) {
            e.printStackTrace();
        }

        ppb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        ppb.Claw.setPosition(AMAX_POS);

        ppb.slideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ppb.slideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (opModeInInit())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if (tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        /* Actually do something useful */
        if(tagOfInterest == null) {
            /*ppb.closeClaw();
            sleep(750);
            ppb.moveSlidesToHeight(JUNCTION_HIGH);
            //Bring the slides up
            ppb.driveStraightGyro(5, 0.5);
            //Move straight to the junction
            ppb.openClaw();
            //Open the claw
            sleep(1200);
            ppb.driveStraightGyro(-5, 0.2);
            //Move back to the center
            ppb.moveSlidesToHeight(JUNCTION_GROUND);
            //Move to cone height
            ppb.strafeRightIMU(12, 0.3);
            //Move back to the center*/
        }

        else if (tagOfInterest != null) {
            ppb.closeClaw();
            sleep(800);
            //Close the Claw
            ppb.driveStraightGyro(27, 0.5);
            //Move to the high junction
            ppb.strafeRightIMU(11.5, 0.4);
            //Align with the center
            ppb.moveSlidesToHeight(JUNCTION_MEDIUM);
            //Bring the slides up
            ppb.driveStraightGyro(5, 0.3);
            //Move straight to the junction
            ppb.openClaw();
            //Open the claw
            sleep(1200);
            ppb.driveStraightGyro(-5, 0.3);
            //Move back to the center
            ppb.moveSlidesToHeight(515);
            //Move to cone height
            ppb.strafeLeftIMU(11, 0.5);
            //Move to the center and forward too far
            ppb.driveStraightGyro(36, 0.5);
            ppb.driveStraightGyro(-9, 0.5);
            //Move back to center
            ppb.turnTankGyro(85, 0.4);
            //Turn to face cone stack
            ppb.driveStraightGyro(20, 0.5);
            ppb.driveStraightGyro(5.5, 0.20);
            //Drive to cone stack
            ppb.closeClaw();
            //Pick up new cone
            ppb.moveSlidesToHeight(JUNCTION_MEDIUM);
            //Move up to the medium junction
            ppb.driveStraightGyro(-7, 0.5);
            //Move back to the center of the square
            ppb.driveStraightGyro(-41.5, 0.7);
            //Move back to the medium junction
            ppb.strafeLeftIMU(11.75, 0.4);
            //Align with the junction
            ppb.moveSlidesToHeight(JUNCTION_MEDIUM);
            //Move up the slides
            ppb.driveStraightGyro(4.5, 0.3);
            //Move to the junction
            ppb.openClaw();
            //Drop the cone
            ppb.driveStraightGyro(-4.5, 0.3);
            //Move to the junction
            ppb.strafeLeftIMU(11.5, 0.5);
            //Center in the parking square one
            ppb.moveSlidesToHeight(100);
            if(tagOfInterest.id == LEFT)
                {
                    ppb.driveStraightGyro(48, 1);
                    sleep(100);
                } //Move to square three

            else if(tagOfInterest.id == MIDDLE)
                {
                    ppb.driveStraightGyro(22, 1);
                    sleep(100);
                } //Move to square two

            else if(tagOfInterest.id == RIGHT)
                {
                    sleep(1000);
                } //Stay in that square
            }
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}