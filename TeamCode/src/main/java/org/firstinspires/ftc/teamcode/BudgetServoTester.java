package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp (name = "BudgetServoTester")
public class BudgetServoTester extends LinearOpMode {
    double targetWrist;
    double targetClaw;
    Servo wrist = null;
    Servo claw = null;

    @Override
        public void runOpMode(){
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(),telemetry);
        wrist = hardwareMap.get(Servo.class, "wrist");
        claw = hardwareMap.get(Servo.class, "claw");

        waitForStart();

        while(opModeIsActive()){
            if(gamepad2.dpad_up) targetWrist += 0.0001;
            if(gamepad2.dpad_down) targetWrist -= 0.0001;
            if(gamepad2.right_bumper) targetClaw += 0.0001;
            if(gamepad2.left_bumper) targetClaw -= 0.0001;
            wrist.setPosition(targetWrist);
            claw.setPosition(targetClaw);
            telemetry.addData("wrist", targetWrist);
            telemetry.addData("claw", targetClaw);
            telemetry.update();
        }
    }

}
