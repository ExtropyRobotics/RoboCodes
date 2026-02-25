package org.firstinspires.ftc.teamcode.TeleOPs.newRobot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.List;

@TeleOp(name = "TurretTuning")
public class turretTuning extends LinearOpMode {

    // PID
    private double kP = 0.06;
    private double kI = 0.0;
    private double kD = 0.0;
    private double kF = 0.1;

    private double integral = 0;
    private double lastError = 0;
    private double lastOutput = 0;

    private ElapsedTime timer = new ElapsedTime();

    // limits
    private static final double POSITION_TOLERANCE = 1.5;
    private static final double MAX_POWER = 1;
    private static final double MAX_OUTPUT_CHANGE = 0.05;
    private static final double INTEGRAL_LIMIT = 10;

    // memory
    private double lastTx = 0;
    private double lastTxVelocity = 0;
    private boolean tagVisible = true;

    // hardware
    DcMotorEx turret;
    Limelight3A limelight;
    SampleMecanumDrive drive;

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

        while (opModeIsActive() && !isStopRequested()) {

            drive.setWeightedDrivePower(new Pose2d(
                    gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    gamepad1.right_stick_x
            ));
            drive.update();

            Pose2d poseVelocity = drive.getPoseVelocity();
            double robotOmega = 0;
            if (poseVelocity != null) {
                robotOmega = poseVelocity.getHeading();
            }

            // limelight init
            LLResult result = limelight.getLatestResult();
            boolean currentlyVisible = false;

            double tx = 0;

            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
                if (tags != null && !tags.isEmpty()) {
                    tx = tags.get(0).getTargetXDegrees();
                    currentlyVisible = true;
                }
            }

            if (currentlyVisible) {
                // normal PID control
                tagVisible = true;

                double error = tx;
                double dt = timer.seconds();
                timer.reset();
                if (dt <= 0) dt = 0.02;

                // update vel for future reacquisition
                lastTxVelocity = (tx - lastTx) / dt;
                lastTx = tx;

                if (Math.abs(error) < POSITION_TOLERANCE) {
                    turret.setPower(0);
                    resetController();
                } else {
                    integral += error * dt;
                    integral = Math.max(-INTEGRAL_LIMIT, Math.min(INTEGRAL_LIMIT, integral));
                    double derivative = (error - lastError) / dt;

                    double output = (kP * error) + (kI * integral) + (kD * derivative) + (kF * robotOmega);

                    // clamp output
                    output = Math.max(-MAX_POWER, Math.min(MAX_POWER, output));

                    double delta = output - lastOutput;
                    delta = Math.max(-MAX_OUTPUT_CHANGE, Math.min(MAX_OUTPUT_CHANGE, delta));
                    output = lastOutput + delta;

                    turret.setPower(output);

                    lastError = error;
                    lastOutput = output;
                }

            } else {
                // if tag is lost, keep moving till found
                if (tagVisible) {
                    // keep last tx seen before apriltag out of frame
                    // already stored in previous frame
                }
                tagVisible = false;

                // continuous movement opposite the last observed tag motion
                double searchPower = MAX_POWER * Math.signum(lastTxVelocity);
                turret.setPower(searchPower);
            }

            // tuning
            tuningControls();

            telemetry.addLine("Y/A = kP | Dpad Up/Down = kI | Dpad Right/Left = kD | RB/LB = kF");
            telemetry.addData("kP", kP);
            telemetry.addData("kI", kI);
            telemetry.addData("kD", kD);
            telemetry.addData("kF", kF);
            telemetry.addData("Error", lastError);
            telemetry.addData("TagVisible", currentlyVisible);
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

    private void resetController() {
        integral = 0;
        lastError = 0;
        lastOutput = 0;
    }
}