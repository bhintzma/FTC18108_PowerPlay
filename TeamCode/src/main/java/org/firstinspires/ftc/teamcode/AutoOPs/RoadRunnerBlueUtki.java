package org.firstinspires.ftc.teamcode.AutoOPs;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.apache.commons.math3.analysis.function.Power;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Vision.EasyOpenCVVision1;
import org.firstinspires.ftc.teamcode.DefineRobot.PowerPlayBot;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous
public class RoadRunnerBlueUtki extends LinearOpMode {
    // OpenCvCamera webcam;
    public void runOpMode() {
        /*int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        EasyOpenCVVision1 pipeline = new EasyOpenCVVision1();
        webcam.setPipeline(pipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

         */
        PowerPlayBot ppb = new PowerPlayBot(this, hardwareMap);



        /* Servo s3Rotation = null;
        s3Rotation = hardwareMap.get(Servo.class, "s3 rotation");

        DcMotor m6Intake = null;
        DcMotor m5Lift = null;
        DcMotor m7carousel=null;
        DcMotor m8Val=null;

        m6Intake = hardwareMap.get(DcMotor.class, "m6 intake");
        m5Lift = hardwareMap.get(DcMotor.class, "m5 lift");
        m7carousel = hardwareMap.get(DcMotor.class, "m7 rul");
        m8Val = hardwareMap.get(DcMotor.class, "m8 Val");

         */




        // We want to start the bot at x: 10, y: -8, heading: 90 degrees
        Pose2d startPose = new Pose2d(0, 0, Math.toDegrees(0));


        Trajectory first = ppb.trajectoryBuilder(startPose, true)
                .back(15)
                .build();

        Trajectory firstt = ppb.trajectoryBuilder(first.end(), true)
                .lineToConstantHeading(new Vector2d(-12, 40))
                .build();

        Trajectory second = ppb.trajectoryBuilder(firstt.end(), true)
                .lineToConstantHeading(new Vector2d(7, -13))
                .build();

        Trajectory secondd = ppb.trajectoryBuilder(second.end(), true)
                .strafeRight(2)
                .build();

        Trajectory seconddd = ppb.trajectoryBuilder(secondd.end(), true)
                .forward(1)
                .build();

        Trajectory third = ppb.trajectoryBuilder(seconddd.end(), true)
                .strafeLeft(5)
                .forward(3)
                .strafeRight(4)
                .build();

        Trajectory firsttwo = ppb.trajectoryBuilder(third.end(), true)
                .lineToConstantHeading(new Vector2d(-12, 40))
                .build();

        Trajectory third1 = ppb.trajectoryBuilder(second.end(), true)
                .lineToConstantHeading(new Vector2d(-16, -22))
                .build();


        waitForStart();
        {


            ppb.followTrajectory(firstt);

        }
    }
}
