package org.firstinspires.ftc.teamcode.TeleOPs.Tester;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@TeleOp (name = "portTest")
public class testingControlHub extends LinearOpMode {

    DcMotorEx motorPort0 = null;
    DcMotorEx motorPort1 = null;
    DcMotorEx motorPort2 = null;
    DcMotorEx motorPort3 = null;
    Servo servoPort0 = null;
    Servo servoPort1 = null;
    Servo servoPort2 = null;
    Servo servoPort3 = null;
    Servo servoPort4 = null;
    Servo servoPort5 = null;

    @Override
    public void runOpMode() throws InterruptedException {

        motorPort0 = hardwareMap.get(DcMotorEx.class, "motor0");
        motorPort1 = hardwareMap.get(DcMotorEx.class, "motor1");
        motorPort2 = hardwareMap.get(DcMotorEx.class, "motor2");
        motorPort3 = hardwareMap.get(DcMotorEx.class, "motor3");

        motorPort0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorPort1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorPort2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorPort3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while(opModeIsActive() && !isStopRequested()){




        }
    }
}