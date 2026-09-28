package org.firstinspires.ftc.teamcode.TeleOPs.activitati;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp (name = "workshop")
public class workshop extends LinearOpMode {

    DcMotorEx motor1;
    DcMotorEx motor2;
    Servo servo;
    CRServo crservo;
    int motorPosition = 0;
    double servoPosition = 0;


    @Override
    public void runOpMode() throws InterruptedException {

        motor1 = hardwareMap.get(DcMotorEx.class, "motor1");
        motor2 = hardwareMap.get(DcMotorEx.class, "motor2");
        servo = hardwareMap.get(Servo.class, "servo");
        crservo = hardwareMap.get(CRServo.class, "crservo");

        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setTargetPosition(motorPosition);

        waitForStart();

        while(opModeIsActive() && !isStopRequested()){
            motor1.setTargetPosition(motorPosition);
            motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            if(gamepad1.dpad_up)
                {
                    motorPosition += 1;
                    motor2.setPower(-0.8);
                }
            if(gamepad1.dpad_down)
                {
                    servoPosition -= 0.005;
                    crservo.setPower(1);
                }
            if (gamepad1.dpad_right) {
                motorPosition -= 1;
                servoPosition=0;

            }
            if  (gamepad1.dpad_left){
                motor2.setPower(1);
                crservo.setPower(-1);
            }
            motor1.setPower(1);
            servo.setPosition(servoPosition);

        }
    }
}