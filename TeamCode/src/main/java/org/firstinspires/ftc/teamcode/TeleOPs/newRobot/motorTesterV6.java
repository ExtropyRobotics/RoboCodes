package org.firstinspires.ftc.teamcode.TeleOPs.newRobot;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Disabled
@TeleOp (name = "MotorTesterV6", group = "Tester")
public class motorTesterV6 extends LinearOpMode {
    DcMotorEx motor1;
    DcMotorEx motor2;
    @Override
    public void runOpMode() throws InterruptedException {
        motor1 = hardwareMap.get(DcMotorEx.class, "outtake");
        motor2 = hardwareMap.get(DcMotorEx.class, "outtake2");

        waitForStart();

        while(opModeIsActive()){
            if(gamepad1.dpad_up) motor1.setPower(0.4);
            else motor1.setPower(0);

            if(gamepad1.dpad_down) motor2.setPower(0.4);
            else motor2.setPower(0);
        }
    }
}