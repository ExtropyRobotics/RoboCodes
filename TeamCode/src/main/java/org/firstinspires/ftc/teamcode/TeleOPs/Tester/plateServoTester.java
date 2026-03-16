package org.firstinspires.ftc.teamcode.TeleOPs.Tester;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp (name = "plateServoTester", group = "Tester")
public class plateServoTester extends LinearOpMode {
    CRServo servo1 = null;
    CRServo servo2 = null;

    @Override
    public void runOpMode(){
        servo1 = hardwareMap.get(CRServo.class, "plateServoLeft");
        servo2 = hardwareMap.get(CRServo.class, "plateServoRight");

        waitForStart();

        while(opModeIsActive() && !isStopRequested()){
            if(gamepad1.dpad_up) servo1.setPower(1);
            else servo1.setPower(0);

            if(gamepad1.dpad_down) servo2.setPower(1);
            else servo2.setPower(0);
        }
    }
}
