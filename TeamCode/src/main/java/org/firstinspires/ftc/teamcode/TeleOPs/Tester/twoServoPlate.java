package org.firstinspires.ftc.teamcode.TeleOPs.Tester;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp (name = "servoPlate")
public class twoServoPlate extends LinearOpMode {
    CRServo servoLeft = null;
    CRServo servoRight = null;
    DcMotor intake = null;
    double platePow = 0;
    AnalogInput encoderRight = null;
    AnalogInput encoderLeft = null;
    double rightPose = 0;
    double leftPose = 0;
    boolean motorToggle = false;
    boolean motorOnce = false;
    DcMotorEx outtake = null;
    boolean outtakeToggle = false;
    boolean outtakeOnce = false;

    @Override
    public void runOpMode(){
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(),telemetry);
        servoLeft = hardwareMap.get(CRServo.class, "servoLeft");
        servoRight = hardwareMap.get(CRServo.class, "servoRight");
        encoderRight = hardwareMap.get(AnalogInput.class, "encoderRight");
        encoderLeft = hardwareMap.get(AnalogInput.class, "encoderLeft");
        intake = hardwareMap.get(DcMotor.class, "motor");
        outtake = hardwareMap.get(DcMotorEx.class, "outtake");
        outtake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while(opModeIsActive()){
            leftPose = 1 - encoderLeft.getVoltage() / 3.3;
            rightPose = 1 - encoderRight.getVoltage() / 3.3;

            if(gamepad1.right_bumper) platePow = 1;
            if(gamepad1.left_bumper) platePow = -1;
            if(gamepad1.y) platePow = 0;

            intake.setPower(1);

            if(rightPose >= 0.95 && platePow == 1 || platePow == 0) platePow = 0;
            if(leftPose >= 0.64  && platePow == -1 || platePow == 0) platePow = 0;

            if(gamepad1.a) {
                if (!motorToggle) {
                    if (motorOnce) intake.setDirection(DcMotorSimple.Direction.REVERSE);
                    else intake.setDirection(DcMotorSimple.Direction.FORWARD);
                    motorOnce = !motorOnce;
                    motorToggle = true;
                }
            } else motorToggle = false;

            if(gamepad1.dpad_up) {
                if (!outtakeToggle) {
                    if (outtakeOnce) outtake.setVelocity(0);
                    else outtake.setVelocity(1050);
                    outtakeOnce = !motorOnce;
                    outtakeToggle = true;
                }
            } else outtakeToggle = false;

            if(gamepad1.dpad_down){
                outtakeToggle = false;
                outtakeOnce = false;
                outtake.setVelocity(2000);
            }

            servoLeft.setPower(platePow);
            servoRight.setPower(platePow);

            telemetry.addLine("right/left bumper - plate");
            telemetry.addLine("△ - stop plate");
            telemetry.addLine("dpad up - start/stop outtake");
            telemetry.addLine("dpad down - outtake velocity 2000");
            telemetry.addLine("x - reverse intake");
            telemetry.addData("encoderLeft: ", leftPose);
            telemetry.addData("encoderRight: ", rightPose);
            telemetry.addData("outtake RPM:", outtake.getVelocity());
            telemetry.update();
        }
    }
}
