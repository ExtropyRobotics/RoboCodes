package org.firstinspires.ftc.teamcode.TeleOPs.activitati;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Disabled
@TeleOp (name = "activitate gradinita", group = "Activitati")
public class teleOPgradinita extends LinearOpMode {
    CRServo servoLeft = null;
    CRServo servoRight = null;
    DcMotor intake = null;
    double platePow = 0;
    DcMotorEx outtake = null;
    boolean outtakeToggle = false;
    boolean outtakeOnce = false;
    boolean intakeToggle = false;
    boolean intakeOnce = false;
    SampleMecanumDrive drive;
    DcMotorEx turret;
    int turretTarget = 0;


    @Override
    public void runOpMode(){
        drive = new SampleMecanumDrive(hardwareMap);
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(),telemetry);
        servoLeft = hardwareMap.get(CRServo.class, "plateServoLeft");
        servoRight = hardwareMap.get(CRServo.class, "plateServoRight");
        intake = hardwareMap.get(DcMotor.class, "intake");
        outtake = hardwareMap.get(DcMotorEx.class, "outtake");
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        outtake.setDirection(DcMotorSimple.Direction.REVERSE);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setTargetPosition(turretTarget);

        waitForStart();

        while(opModeIsActive() && !isStopRequested()){
            turret.setPower(1);

            turret.setTargetPosition(turretTarget);

            drive.setWeightedDrivePower(new Pose2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x
            ));

            if(gamepad1.right_bumper) platePow = 0.5;
            if(gamepad1.left_bumper) platePow = -0.5;
            if(gamepad1.y) platePow = 0; // △


            if(gamepad1.b) { // ◯
                if (!outtakeToggle) {
                    if (outtakeOnce) outtake.setPower(0);
                    else outtake.setPower(1);
                    outtakeOnce = !outtakeOnce;
                    outtakeToggle = true;
                }
            } else outtakeToggle = false;

            if(gamepad1.x) { // □
                if (!intakeToggle) {
                    if (intakeOnce) intake.setPower(0);
                    else intake.setPower(1);
                    intakeOnce = !intakeOnce;
                    intakeToggle = true;
                }
            } else intakeToggle = false;

            servoLeft.setPower(platePow);
            servoRight.setPower(platePow);

            if(gamepad1.dpad_left) turretTarget += 1;
            if(gamepad1.dpad_right) turretTarget -= 1;

            turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            telemetry.addLine("Right Bumper - rotate plate");
            telemetry.addLine("Left Bumper - shoot");
            telemetry.addLine("△ - stop plate entirely");
            telemetry.addLine("◯ - start/stop outtake");
            telemetry.addLine("□ - start/stop intake");
            telemetry.addLine("dpads to move turret");
            telemetry.update();
        }
    }
}
