package org.firstinspires.ftc.teamcode.TeleOPs;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

//@TeleOp (name = "TeleOP")
public class TeleOP extends LinearOpMode {
    private DcMotor launcher;

    SampleMecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {

        launcher = hardwareMap.get(DcMotor.class, "plate");

        waitForStart();

        while(opModeIsActive()){

            if(gamepad2.a){
                launcher.setPower(1);
            } else launcher.setPower(0);
        }
    }
}
