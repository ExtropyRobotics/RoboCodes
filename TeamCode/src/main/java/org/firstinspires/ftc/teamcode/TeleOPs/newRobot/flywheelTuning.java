package org.firstinspires.ftc.teamcode.TeleOPs.newRobot;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Disabled
@TeleOp (name = "flywheelTuning", group = "TeleOpTuning")
public class flywheelTuning extends OpMode {
    public DcMotorEx flywheelMotor;
    public double highVelocity = 1300;
    public double lowVelocity = 1200;

    double curTargetVelocity = highVelocity;

    double flywheelF = 0;
    double flywheelP = 0;

    double[] stepSizes = {10.0, 1.0, 0.1, 0.01, 0.001, 0.0001};

    int stepIndex = 1;
    @Override
    public void init() {
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "outtake");
        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(flywheelP, 0, 0, flywheelF);
        flywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        telemetry.addLine("Init Completed");
    }

    @Override
    public void loop() {

        if(gamepad1.yWasPressed()){
            if(curTargetVelocity == highVelocity){
                curTargetVelocity = lowVelocity;
            } else curTargetVelocity = highVelocity;
        }

        if(gamepad1.bWasPressed()){
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        if(gamepad1.dpadLeftWasPressed()){
            flywheelF -= stepSizes[stepIndex];
        }

        if(gamepad1.dpadRightWasPressed()){
            flywheelF += stepSizes[stepIndex];
        }

        if(gamepad1.dpadUpWasPressed()){
            flywheelP += stepSizes[stepIndex];
        }

        if(gamepad1.dpadDownWasPressed()){
            flywheelP -= stepSizes[stepIndex];
        }

        // set new PIDF coefficients
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(flywheelP, 0, 0, flywheelF);
        flywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        flywheelMotor.setVelocity(curTargetVelocity);

        double curVelocity = flywheelMotor.getVelocity();
        double error = curTargetVelocity - curVelocity;

        telemetry.addData("Target Velocity", curTargetVelocity);
        telemetry.addData("Current Velocity", "%.2f", curVelocity);
        telemetry.addData("Error", "%.2f", error);
        telemetry.addLine("-------------------------------");
        telemetry.addData("Tuning P", "%.4f (D-Pad U/D)", flywheelP);
        telemetry.addData("Tuning F", "%.4f D-Pad L/R", flywheelF);
        telemetry.addData("Step size", "%.4f (B-Button)", stepSizes[stepIndex]);
        // https://www.youtube.com/watch?v=aPNCpZzCTKg
    }
}
