package org.firstinspires.ftc.teamcode.TeleOPs.oldRobot.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@Disabled
@TeleOp (name = "testSpinner", group = "Tester")
public class testSpinner extends LinearOpMode {

    DcMotor motor;

    @Override
    public void runOpMode() throws InterruptedException {
        motor = hardwareMap.get(DcMotor.class, "motor");
        waitForStart();
        while(opModeIsActive()){
            motor.setPower(1);
        }
    }
}