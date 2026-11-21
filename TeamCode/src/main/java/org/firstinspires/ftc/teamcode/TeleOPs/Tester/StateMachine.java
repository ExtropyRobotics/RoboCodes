package org.firstinspires.ftc.teamcode.TeleOPs.Tester;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp (name = "test")
public class StateMachine extends LinearOpMode {

    public enum stateList{
        Motor,
        Servo,
        test
    }

    DcMotorEx motor = null;
    CRServo servo;
    stateList state = stateList.Motor;

    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        motor = hardwareMap.get(DcMotorEx.class, "motor");
        servo = hardwareMap.get(CRServo.class, "servo");

        timer.reset();

        waitForStart();

        while(opModeIsActive() && !isStopRequested()){

           switch (state){
               case Motor:

                   servo.setPower(0);
                   motor.setPower(1);

                   break;


               case Servo:

                   motor.setPower(0);
                   servo.setPower(1);

                   break;

               case test:

                   if(timer.seconds() < 0.5){
                       motor.setPower(0);
                       servo.setPower(0);
                   }

                   if(timer.seconds() > 0.5 && timer.seconds() < 0.9){
                       motor.setPower(0);
                       servo.setPower(1);
                   }

                   if(timer.seconds() > 1){
                       motor.setPower(1);
                       servo.setPower(0);
                   }

                   break;
           }

            if(gamepad1.dpad_up) state = stateList.Motor;
            if(gamepad1.dpad_down) state = stateList.Servo;

            if(gamepad1.y){
                state = stateList.test;
                timer.reset();
            }

            telemetry.addData("state", state);
            telemetry.update();

        }
    }
}