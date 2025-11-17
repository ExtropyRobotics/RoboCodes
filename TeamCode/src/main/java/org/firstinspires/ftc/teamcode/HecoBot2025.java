package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp (name = "!! Shooter")
public class HecoBot2025 extends LinearOpMode {
    SampleMecanumDrive drive;
    DcMotor wheel1;
    DcMotor wheel2;
    DcMotor ax;
    Servo spinn;
    boolean toggleSpin = false;
    double spin = 0;

    @Override
    public void runOpMode() throws InterruptedException{
        drive = new SampleMecanumDrive(hardwareMap);

        wheel1 = hardwareMap.get(DcMotor.class, "wheel1");
        wheel2 = hardwareMap.get(DcMotor.class, "wheel2");
        ax = hardwareMap.get(DcMotor.class, "ax");
        spinn = hardwareMap.get(Servo.class, "belt");

        waitForStart();

        while(opModeIsActive()){
            drive.setWeightedDrivePower(new Pose2d(
                    gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    -gamepad1.right_stick_x
            ));

            if(gamepad1.a){
                wheel1.setPower(1);
                wheel2.setPower(-1);
            }
            if(gamepad1.b){
                wheel1.setPower(0);
                wheel2.setPower(0);
            }

            if(gamepad1.dpad_right && !toggleSpin){
                    spin += 0.05;
                    if(spin > 1) spin = 0;
                    if(spin == 0.05) spin = 0.35;
                    if(spin >= 0.75) spin = 0.35;
                    spinn.setPosition(spin);

            } toggleSpin = gamepad1.dpad_right;

            if(gamepad1.dpad_up) {
                ax.setPower(-0.4);
            }else if(gamepad1.dpad_down){
                ax.setPower(0.4);
            } else ax.setPower(0);
        }
    }
}