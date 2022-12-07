package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.DefineRobot.PowerPlayBot;


/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(group = "drive")
public class TeleOp_Drive extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        PowerPlayBot ppb = new PowerPlayBot(this, hardwareMap);

        try {
            ppb.init();
        } catch (Exception e) {
            e.printStackTrace();
        }

        ppb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ppb.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (!isStopRequested()) {

            ppb.mecanumDriving();

            if (gamepad2.left_bumper)
                ppb.openClaw();
            else if (gamepad2.right_bumper)
                ppb.closeClaw();

            if (gamepad2.y)
                ppb.moveSlidesToHeight(4000);

            ppb.moveSlides();

            ppb.update();

            Pose2d poseEstimate = ppb.getPoseEstimate();
            telemetry.addData("sl: " , ppb.slideLeft.getCurrentPosition());
            telemetry.addData("sr: " , ppb.slideRight.getCurrentPosition());
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("servoPosition", ppb.Aposition);
            telemetry.update();
        }
    }
}
