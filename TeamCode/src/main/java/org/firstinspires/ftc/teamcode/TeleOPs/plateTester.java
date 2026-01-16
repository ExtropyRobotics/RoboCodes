package org.firstinspires.ftc.teamcode.TeleOPs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp (name = "plateTester")
public class plateTester extends LinearOpMode {

    DcMotor plate;
    Servo servoRight;
    Servo servoLeft;
    int targetPoz = 0;
    boolean slightAdd = false;
    boolean slightRemove = false;
    boolean resetEncoder = false;

    @Override
    public void runOpMode() throws InterruptedException {

        plate = hardwareMap.get(DcMotor.class, "plate");
        servoLeft = hardwareMap.get(Servo.class, "servoLeft");
        servoRight = hardwareMap.get(Servo.class, "servoRight");

        plate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        plate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        plate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        plate.setPower(1);
        plate.setTargetPosition(targetPoz);
        plate.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while(opModeInInit() && !isStopRequested()){

            servoRight.setPosition(0.4592);
            servoLeft.setPosition(0.8114);

            telemetry.addData("targetpoz", targetPoz);
            telemetry.update();
        }

        waitForStart();

        while(opModeIsActive() && !isStopRequested()){

            if(!resetEncoder){
                plate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                resetEncoder = true;
            }

            servoRight.setPosition(1);
            servoLeft.setPosition(0.7294);


            if(gamepad1.dpad_up){
                targetPoz += 1;
            }

            if(gamepad1.dpad_down){
                targetPoz -= 1;
            }


            if(gamepad1.y && !slightAdd){
                targetPoz += 179;
                slightAdd = true;
            } else slightAdd = false;

            if(gamepad1.a && !slightRemove){
                targetPoz -= 179;
                slightRemove = true;
            } else slightRemove = false;



            plate.setTargetPosition(targetPoz);

            telemetry.addData("targetpoz", targetPoz);
            telemetry.update();
        }
    }
}