package org.firstinspires.ftc.teamcode.TeleOPs.newRobot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.List;

@Disabled
@TeleOp(name = "TurretTuning", group = "TeleOpTuning")
public class turretTuning extends LinearOpMode {

    // PID
    private double kP = 0.02;
    private double kI = 0.0;
    private double kD = 0.0;
    private double kF = 0.0;

    private double turretIntegral = 0;
    private double turretLastError = 0;
    private double turretLastOutput = 0;
    double error = 0;

    private ElapsedTime timer = new ElapsedTime();

    // limits
    int MAX_TURRET_TICKS = 2100;
    int MIN_TURRET_TICKS = -230;
    private static final double POSITION_TOLERANCE = 1.5;
    private static final double MAX_POWER = 1;
    private static final double MAX_OUTPUT_CHANGE = 0.05;
    private static final double INTEGRAL_LIMIT = 10;
    int currentPos = 0;

    // memory
    double tx = 0;
    private double lastTx = 0;
    private double lastTxVelocity = 0;
    double searchPower = 0;
    // hardware
    DcMotorEx turret;
    Limelight3A limelight;
    SampleMecanumDrive drive;
    boolean unwinding = false;

    // toggles
    boolean kpUp, kpDown, kiUp, kiDown, kdUp, kdDown, kfUp, kfDown;


    @Override
    public void runOpMode() {

        drive = new SampleMecanumDrive(hardwareMap);
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        turret = hardwareMap.get(DcMotorEx.class, "turret");

        turret.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        limelight.pipelineSwitch(1);
        limelight.start();

        waitForStart();
        timer.reset();
        double dt = timer.seconds();

        while (opModeIsActive() && !isStopRequested()) {
            currentPos = turret.getCurrentPosition();

            lastTxVelocity = (error - lastTx) / dt;
            searchPower =  0.4 * Math.signum(error);
            if (dt <= 0) dt = 0.02;

            Pose2d poseVelocity = drive.getPoseVelocity();
            double robotOmega = 0;
            if (poseVelocity != null) {
                robotOmega = poseVelocity.getHeading();
            }

            if (unwinding) {
                int error = -currentPos;

                if (Math.abs(error) < 200) {
                    turret.setPower(0);
                    unwinding = false;
                    turretIntegral = 0;
                    turretLastError = 0;
                } else {
                    double power = 0.4 * Math.signum(error);
                    turret.setPower(power);
                }
                continue;
            }

            if (currentPos >= MAX_TURRET_TICKS || currentPos <= MIN_TURRET_TICKS) {
                unwinding = true;
                continue;
            }

            // limelight init
            LLResult result = limelight.getLatestResult();
            boolean currentlyVisible = false;

            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
                if (tags != null && !tags.isEmpty()) {
                    tx = tags.get(0).getTargetXDegrees();
                    currentlyVisible = true;
                }
            }

            if (currentlyVisible && !unwinding) {
                // normal PID control
                timer.reset();
                // update vel for future reacquisition
                error = tx;

                if (Math.abs(error) < POSITION_TOLERANCE) {
                    turret.setPower(0);
                    turretIntegral = 0;
                    turretLastError = 0;
                    turretLastOutput = 0;
                } else {
                    turretIntegral += error * dt;
                    turretIntegral = Math.max(-INTEGRAL_LIMIT, Math.min(INTEGRAL_LIMIT, turretIntegral));
                    double derivative = (error - turretLastError) / dt;

                    double output = (kP * error) + (kI * turretIntegral) + (kD * derivative) + (kF * robotOmega);

                    // clamp output
                    output = Math.max(-MAX_POWER, Math.min(MAX_POWER, output));

                    double delta = output - turretLastOutput;
                    delta = Math.max(-MAX_OUTPUT_CHANGE, Math.min(MAX_OUTPUT_CHANGE, delta));
                    output = turretLastOutput + delta;

                    turret.setPower(output);

                    turretLastError = error;
                    turretLastOutput = output;
                    lastTx = tx;
                }

            }
            if (!currentlyVisible && !unwinding){
                // continuous movement opposite the last observed tag motion
                turret.setPower(searchPower);
            }

            // tuning
            tuningControls();

            telemetry.addLine("Y/A = kP | Dpad Up/Down = kI | Dpad Right/Left = kD | RB/LB = kF");
            telemetry.addData("kP", kP);
            telemetry.addData("kI", kI);
            telemetry.addData("kD", kD);
            telemetry.addData("kF", kF);
            telemetry.addData("Error", turretLastError);
            telemetry.addData("lastTx", lastTx);
            telemetry.addData("dt", dt);
            telemetry.addData("TagVisible", currentlyVisible);
            telemetry.addData("searchPower", searchPower);
            telemetry.addData("unwinding", unwinding);
            telemetry.addData("positon", turret.getCurrentPosition());
            telemetry.update();
        }
    }

    private void tuningControls() {
        if (gamepad1.y && !kpUp) { kP += 0.001; kpUp = true; }
        else if (!gamepad1.y) kpUp = false;

        if (gamepad1.a && !kpDown) { kP -= 0.001; kpDown = true; }
        else if (!gamepad1.a) kpDown = false;

        if (gamepad1.dpad_up && !kiUp) { kI += 0.001; kiUp = true; }
        else if (!gamepad1.dpad_up) kiUp = false;

        if (gamepad1.dpad_down && !kiDown) { kI -= 0.001; kiDown = true; }
        else if (!gamepad1.dpad_down) kiDown = false;

        if (gamepad1.dpad_right && !kdUp) { kD += 0.001; kdUp = true; }
        else if (!gamepad1.dpad_right) kdUp = false;

        if (gamepad1.dpad_left && !kdDown) { kD -= 0.001; kdDown = true; }
        else if (!gamepad1.dpad_left) kdDown = false;

        if (gamepad1.right_bumper && !kfUp) { kF += 0.001; kfUp = true; }
        else if (!gamepad1.right_bumper) kfUp = false;

        if (gamepad1.left_bumper && !kfDown) { kF -= 0.001; kfDown = true; }
        else if (!gamepad1.left_bumper) kfDown = false;

        kP = Math.max(0, kP);
        kI = Math.max(0, kI);
        kD = Math.max(0, kD);
        kF = Math.max(0, kF);
    }
}