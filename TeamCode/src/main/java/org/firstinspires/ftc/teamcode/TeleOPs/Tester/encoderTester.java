package org.firstinspires.ftc.teamcode.TeleOPs.Tester;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Disabled
@TeleOp (name = "encoderTester", group = "Tester")
public class encoderTester extends LinearOpMode {
    DcMotorEx motor;
    int targetPoz = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        motor = hardwareMap.get(DcMotorEx.class, "motor");
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);

        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setTargetPosition(targetPoz);

        waitForStart();

        while(opModeIsActive()){
            motor.setPower(1);
            motor.setTargetPosition(targetPoz);

            if(gamepad1.dpad_up) targetPoz += 1;
            if(gamepad1.dpad_down) targetPoz -= 1;

            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            telemetry.addData("ticks:", targetPoz);
            telemetry.update();
        }
    }
}