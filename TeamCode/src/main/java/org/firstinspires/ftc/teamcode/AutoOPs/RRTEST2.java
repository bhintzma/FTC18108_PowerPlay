package org.firstinspires.ftc.teamcode.AutoOPs;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


import org.apache.commons.math3.analysis.function.Pow;
import org.apache.commons.math3.analysis.function.Power;
import org.firstinspires.ftc.teamcode.Vision.EasyOpenCVVision1;
import org.firstinspires.ftc.teamcode.DefineRobot.PowerPlayBot;

@Autonomous
public class RRTEST2 extends LinearOpMode {
    public void runOpMode() {

        PowerPlayBot ppb = new PowerPlayBot(this, hardwareMap);

        try {
            ppb.init();
        } catch (Exception e) {
            e.printStackTrace();
        }

        ppb.runtime.reset();

        // We want to start the bot at x: 10, y: -8, heading: 90 degrees
        Pose2d startPose = new Pose2d(0, 0, Math.toDegrees(0));


        Trajectory first = ppb.trajectoryBuilder(startPose, true)
                .back(15)
                .build();

        Trajectory firstt = ppb.trajectoryBuilder(startPose, true)
                .lineToConstantHeading(new Vector2d(-25, 34))
                .build();


        Trajectory secondd = ppb.trajectoryBuilder(startPose, false)
                .lineToLinearHeading(new Pose2d(-25, 34, Math.toDegrees(-90)))
                .build();

        Trajectory second = ppb.trajectoryBuilder(secondd.end(), Math.toDegrees(-90))
                .strafeTo(new Vector2d(-20, 35))
                //.strafeLeft(40)
                .build();

        Trajectory third = ppb.trajectoryBuilder(second.end(), false)
                .forward(40)
                .build();




        waitForStart();
        {

            ppb.followTrajectory(firstt);
            //sleep(1000);
            /* ppb.setMotorPowers(0.3,0.3,0.3,0.3);
            sleep(2000);
            ppb.setMotorPowers(0,0,0,0);
            ppb.setMotorPowers(0.5,-0.5,-0.5,0.5);
            sleep(1000);

             */

        }
    }
}
