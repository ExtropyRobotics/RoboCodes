package org.firstinspires.ftc.teamcode.TeleOPs.Tester;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp (name = "BudgetServoTester", group = "Tester")
public class BudgetServoTester extends LinearOpMode {
    Servo servo = null;
    double servoPoz = 0.5;

    @Override
    public void runOpMode(){
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(),telemetry);
        servo = hardwareMap.get(Servo.class, "servo");

        waitForStart();

        while(opModeIsActive()){
            if(gamepad2.dpad_up) servoPoz += 0.0001;
            if(gamepad2.dpad_down) servoPoz -= 0.0001;

            if(gamepad2.y) servoPoz = 1;
            if(gamepad2.a) servoPoz = 0.55;

            servo.setPosition(servoPoz);

            telemetry.addData("servo: ", servoPoz);
            telemetry.update();
        }
    }
}
    