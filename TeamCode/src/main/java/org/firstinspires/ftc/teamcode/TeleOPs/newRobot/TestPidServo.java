package org.firstinspires.ftc.teamcode.TeleOPs.newRobot;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class TestPidServo extends LinearOpMode{


    CRServo servo = null;
    AnalogInput encoder = null;
    DcMotor motorEncoder = null;


    double pos = 0.5;
    int encoderPos = 0;
    double givenPos = 5;
    double error = 0;

    double p = 0.0005;
    double i = 0.00001;

    int increment = 0;

    int targetPos = 0;
    int sum = 0;


    public void runOpMode(){

        servo = hardwareMap.get(CRServo.class, "servo");
        encoder = hardwareMap.get(AnalogInput.class, "encoder");
        motorEncoder = hardwareMap.get(DcMotor.class, "motor");
        motorEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        waitForStart();

        while(opModeIsActive()){
            encoderPos = motorEncoder.getCurrentPosition();
            error = targetPos - encoderPos;
            sum += (int) error;

            pos = p*error + i*sum;

            if(gamepad1.left_stick_y > 0) targetPos += 10;
            if(gamepad1.left_stick_y < 0) targetPos -= 10;

            if(pos < -1) pos = -1;
            if(pos > 1) pos = 1;


            motorEncoder.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            servo.setPower(pos);

            telemetry.addData("revEncoderPos:", targetPos);
            telemetry.addData("increment", increment);
            telemetry.addData("voltage", encoder.getVoltage());
            telemetry.addData("error", error);
            telemetry.addData("pos", pos);
            telemetry.addData("given Position", givenPos);
            telemetry.addData("given Position but voltage", givenPos/2.24);
            telemetry.addData("encoder Position", encoderPos);
            telemetry.update();

        }

    }
}