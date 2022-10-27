package org.firstinspires.ftc.teamcode.AutoOPs;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Vision.EasyOpenCVVision1;
import org.firstinspires.ftc.teamcode.DefineRobot.PowerPlayBot;
import org.firstinspires.ftc.teamcode.Vision.EasyOpenCVVision1;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous
public class RoadRunnerRedUtki extends LinearOpMode {
    OpenCvCamera webcam;
    TouchSensor touch;
    private ElapsedTime lifttime = new ElapsedTime(5);
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
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
        PowerPlayBot drive = new PowerPlayBot(this, hardwareMap);

        Servo s3Rotation = null;
        s3Rotation = hardwareMap.get(Servo.class, "s3 rotation");

        DcMotor m6Intake = null;
        DcMotor m5Lift = null;
        DcMotor m7carousel=null;
        DcMotor m8Val=null;

        touch = hardwareMap.get(TouchSensor.class, "Touch");

        m6Intake = hardwareMap.get(DcMotor.class, "m6 intake");
        m5Lift = hardwareMap.get(DcMotor.class, "m5 lift");
        m7carousel = hardwareMap.get(DcMotor.class, "m7 rul");
        m8Val = hardwareMap.get(DcMotor.class, "m8 Val");




        // We want to start the bot at x: 10, y: -8, heading: 90 degrees
        Pose2d startPose = new Pose2d(0, 0, Math.toDegrees(0));


        Trajectory first = drive.trajectoryBuilder(startPose, true)
                .back(15)
                .build();

        Trajectory firstt = drive.trajectoryBuilder(first.end(), true)
                .lineToConstantHeading(new Vector2d(-16, -37))
                .build();

        Trajectory firstt2 = drive.trajectoryBuilder(firstt.end(), true)
                .back(2)
                .build();

        Trajectory second = drive.trajectoryBuilder(firstt.end(), true)
                .lineToConstantHeading(new Vector2d(6, 18))
                .build();

        Trajectory secondd = drive.trajectoryBuilder(firstt.end(), true)
                .strafeTo(new Vector2d(3,18))
                .build();

        Trajectory seconddd = drive.trajectoryBuilder(secondd.end(), true)
                .strafeLeft(5.5)
                .build();

        Trajectory seconddd1 = drive.trajectoryBuilder(seconddd.end(), true)
                .forward(3)
                .build();

        Trajectory seconddd2 = drive.trajectoryBuilder(seconddd1.end(), true)
                .strafeLeft(1)
                .build();

        Trajectory third = drive.trajectoryBuilder(seconddd2.end(), true)
                .strafeTo(new Vector2d(0,12))
                .build();

        Trajectory forr1 = drive.trajectoryBuilder(third.end(), true)
                .forward(6)
                .build();

        Trajectory forr = drive.trajectoryBuilder(forr1.end(), true)
                .strafeLeft(5)
                .build();

        Trajectory firsttwo = drive.trajectoryBuilder(forr.end(), true)
                .strafeTo(new Vector2d(-16,-37))
                .build();

        Trajectory firsttw0 = drive.trajectoryBuilder(firsttwo.end(), true)
                .back(1)
                .build();

        Trajectory third1 = drive.trajectoryBuilder(firsttw0.end(), true)
                .lineToConstantHeading(new Vector2d(-15, 16))
                .build();


        double BatteryVoltage;

        double result = Double.POSITIVE_INFINITY;
            for (VoltageSensor sensor : hardwareMap.voltageSensor) {
                double voltage = sensor.getVoltage();
                if (voltage > 0) {
                    result = Math.min(result, voltage);
                }
            }


        double voltage = result;
        double koeff = 13.0 / voltage;
        koeff = Math.pow(koeff, 1.25);

        waitForStart();
        {
            int downpos = 1;

            telemetry.addData("DownPos", downpos);
            telemetry.update();
        }


    }
}
