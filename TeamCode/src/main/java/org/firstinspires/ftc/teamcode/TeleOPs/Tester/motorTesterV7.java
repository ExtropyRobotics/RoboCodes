package org.firstinspires.ftc.teamcode.TeleOPs.Tester;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Disabled
@TeleOp (name = "MotorTester7", group = "Tester")
public class motorTesterV7 extends LinearOpMode {
    DcMotorEx motor1;
    DcMotorEx motor2;
    @Override
    public void runOpMode() throws InterruptedException {
        motor1 = hardwareMap.get(DcMotorEx.class, "outtake");
        motor2 = hardwareMap.get(DcMotorEx.class, "outtake2");
        waitForStart();

        while(opModeIsActive()){
            if(gamepad1.dpad_up){
                motor1.setPower(1);
            } else motor1.setPower(0);

            if(gamepad1.dpad_down){
                motor2.setPower(1);
            } else motor2.setPower(0);

            telemetry.addData("RPM1:", motor1.getVelocity());
            telemetry.addData("RPM2:", motor2.getVelocity());
            telemetry.addData("Direction1", motor1.getDirection());
            telemetry.addData("Direction2", motor2.getDirection());
            telemetry.update();
        }
    }
}