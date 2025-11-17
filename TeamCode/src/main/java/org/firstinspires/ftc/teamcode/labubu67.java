package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp (name = "labubu67")
    public class labubu67 extends LinearOpMode {

    DcMotor launcherLeft;
    DcMotor intake;
    DcMotor launcherRight;

    SampleMecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        launcherLeft = hardwareMap.get(DcMotor.class, "launcherLeft");
        launcherRight = hardwareMap.get(DcMotor.class, "launcherRight");
        intake = hardwareMap.get(DcMotor.class, "intake");
        drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        while(opModeIsActive()){
            drive.setWeightedDrivePower(new Pose2d(
                    gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    gamepad1.right_stick_x
            ));

            if(gamepad1.dpad_right){
                launcherRight.setPower(1);
                launcherLeft.setPower(-1);
            } else {
                launcherRight.setPower(0);
                launcherLeft.setPower(0);
            }
            if(gamepad1.dpad_left){
                intake.setPower(-0.6);
            }
            else  {
                intake.setPower(0);
            }
        }

    }
}
