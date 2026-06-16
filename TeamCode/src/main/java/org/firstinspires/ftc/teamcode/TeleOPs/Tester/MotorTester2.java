package org.firstinspires.ftc.teamcode.TeleOPs.Tester;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Disabled
@TeleOp (name = "MotorTester", group = "Tester")
public class MotorTester2 extends LinearOpMode {
    DcMotorEx motor;
    @Override
    public void runOpMode() throws InterruptedException {
        motor = hardwareMap.get(DcMotorEx.class, "motor");
        waitForStart();

        while(opModeIsActive()){
            motor.setPower(1);

            if(gamepad1.right_bumper) motor.setDirection(DcMotorSimple.Direction.REVERSE);
            if(gamepad1.left_bumper) motor.setDirection(DcMotorSimple.Direction.FORWARD);

            telemetry.addData("RPM1:", motor.getVelocity());
            telemetry.update();
        }
    }
}