package org.firstinspires.ftc.teamcode.TeleOPs;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp (name = "BudgetServoTester")
public class BudgetServoTester extends LinearOpMode {
    Servo servo = null;
    Servo servo2 = null;
    double servoPoz = 0.5;
    double servo2Poz = 0.5;

    @Override
    public void runOpMode(){
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(),telemetry);
        servo = hardwareMap.get(Servo.class, "servo");
        servo2 = hardwareMap.get(Servo.class, "servo2");

        waitForStart();

        while(opModeIsActive()){

            if(gamepad2.dpad_up) servoPoz += 0.0001/5; // right
            if(gamepad2.dpad_down) servoPoz -= 0.0001/5;

            if(gamepad2.y) servo2Poz += 0.0001; // left
            if(gamepad2.a) servo2Poz -= 0.0001;

            servo.setPosition(servoPoz);
            servo2.setPosition(servo2Poz);

            telemetry.addData("servo right: ", servoPoz);
            telemetry.addData("servo left: ", servo2Poz);
            telemetry.update();
        }
    }
}