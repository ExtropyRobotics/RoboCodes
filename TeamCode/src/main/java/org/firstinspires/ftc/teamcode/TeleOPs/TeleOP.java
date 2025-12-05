package org.firstinspires.ftc.teamcode.TeleOPs;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp (name = "TeleOP")
public class TeleOP extends LinearOpMode {
    DcMotor intake;
    DcMotor plate;
    SampleMecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        plate = hardwareMap.get(DcMotor.class, "Farfurie");
        intake = hardwareMap.get(DcMotor.class, "Intake");
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