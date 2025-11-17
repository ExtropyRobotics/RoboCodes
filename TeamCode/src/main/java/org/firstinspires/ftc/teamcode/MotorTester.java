package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp (name = "MotorTester")
    public class MotorTester extends LinearOpMode {
    DcMotor m1 = null;
    DcMotor m2 = null;
    DcMotor m3 = null;
    DcMotor m4 = null;

    int m1pos = 0;
    int m2pos = 0;
    int m3pos = 0;
    int m4pos = 0;

    @Override
    public void runOpMode(){
        m1 = hardwareMap.get(DcMotor.class, "m1");
        m2 = hardwareMap.get(DcMotor.class, "m2");
        m3 = hardwareMap.get(DcMotor.class, "m3");
        m4 = hardwareMap.get(DcMotor.class, "m4");

        initializeMotor(m1);
        initializeMotor(m2);
        initializeMotor(m3);
        initializeMotor(m4);

        waitForStart();

        while(opModeIsActive()){
            m1.setPower(1);
            m2.setPower(1);
            m3.setPower(1);
            m4.setPower(1);

            m1.setTargetPosition(m1pos);
            m2.setTargetPosition(m2pos);
            m3.setTargetPosition(m3pos);
            m4.setTargetPosition(m4pos);

            if(gamepad1.dpad_up) m1pos += 1;
            if(gamepad1.dpad_down) m2pos += 1;
            if(gamepad1.dpad_left) m3pos += 1;
            if(gamepad1.dpad_right) m4pos += 1;

            m1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            m2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            m3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            m4.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            telemetry.addData("m1pos:", m1pos);
            telemetry.addData("m2pos:", m2pos);
            telemetry.addData("m3pos:", m3pos);
            telemetry.addData("m4pos:", m4pos);
            telemetry.update();
        }
    }
    private void initializeMotor(DcMotor motor){
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setTargetPosition(0);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}