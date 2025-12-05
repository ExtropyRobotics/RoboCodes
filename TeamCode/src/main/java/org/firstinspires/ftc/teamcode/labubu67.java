package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp (name = "labubu67")
    public class labubu67 extends LinearOpMode {

    DcMotor Intake;
    DcMotor Farfurie;

    SampleMecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        Farfurie = hardwareMap.get(DcMotor.class, "Farfurie");
        Intake = hardwareMap.get(DcMotor.class, "Intake");
        drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        while(opModeIsActive()){
            drive.setWeightedDrivePower(new Pose2d(
                    gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    gamepad1.right_stick_x
            ));
        }

    }
}
