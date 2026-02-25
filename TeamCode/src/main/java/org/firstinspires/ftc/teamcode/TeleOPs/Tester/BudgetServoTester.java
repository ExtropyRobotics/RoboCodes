package org.firstinspires.ftc.teamcode.TeleOPs.Tester;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp (name = "BudgetServoTester")
public class BudgetServoTester extends LinearOpMode {
    Servo servo = null;
    AnalogInput encoder;
    double encoderPoz = 0;
    double servoPoz = 0.5;

    @Override
    public void runOpMode(){
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(),telemetry);
        servo = hardwareMap.get(Servo.class, "servo");
        encoder = hardwareMap.get(AnalogInput.class, "encoder");

        waitForStart();

        while(opModeIsActive()){

            encoderPoz = 1 - encoder.getVoltage() / 3.3;

            if(gamepad2.dpad_up) servoPoz += 0.0001;
            if(gamepad2.dpad_down) servoPoz -= 0.0001;

            if(gamepad2.y) servoPoz = 1;
            if(gamepad2.a) servoPoz = 0.55;

            servo.setPosition(servoPoz);

            telemetry.addData("servo: ", servoPoz);
            telemetry.addData("encoder: ", encoderPoz);
            telemetry.update();
        }
    }
}
    