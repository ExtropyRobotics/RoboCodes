package org.firstinspires.ftc.teamcode.TeleOPs.oldRobot.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Disabled
@TeleOp (name = "newPlateTest", group = "Tester")
public class newPlateTest extends LinearOpMode {
    DcMotorEx motor;
    DcMotor plate;
    @Override
    public void runOpMode() throws InterruptedException {
        motor = hardwareMap.get(DcMotorEx.class, "outtake");
        plate = hardwareMap.get(DcMotor.class, "plate");
        waitForStart();

        while(opModeIsActive()){
            plate.setPower(1);
            motor.setPower(-1);
            if(gamepad1.right_bumper){
                plate.setDirection(DcMotorSimple.Direction.REVERSE);
            }
            if(gamepad1.left_bumper){
                plate.setDirection(DcMotorSimple.Direction.FORWARD);
            }

        }
    }
}