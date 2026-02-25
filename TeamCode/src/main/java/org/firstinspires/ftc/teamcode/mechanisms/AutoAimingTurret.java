package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

@TeleOp(name = "Auto-aiming turret", group = "Iterative Opmode")
public class AutoAimingTurret extends OpMode {

    // Hardware
    private DcMotorEx turretMotor;
    private Limelight3A limelight;

    // PID
    private double kP = 0.015;
    private double kI = 0.0001;
    private double kD = 0.002;

    private double integral = 0;
    private double lastError = 0;

    private ElapsedTime timer = new ElapsedTime();

    // Encoder limits (300° total ≈ ±150°)
    private static final int MAX_TURRET_TICKS = 600;
    private static final int MIN_TURRET_TICKS = -1700;

    private static final int CENTER_POSITION = 0;
    private static final double UNWIND_POWER = 0.9;

    private static final double POSITION_TOLERANCE = 1.5;
    private static final double MAX_POWER = 0.9;
    private static final double MIN_POWER = 0.05;

    private boolean unwinding = false;

    @Override
    public void init() {

        turretMotor = hardwareMap.get(DcMotorEx.class, "aim");

        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        timer.reset();

        telemetry.addLine("Turret Initialized - START CENTERED");
    }

    @Override
    public void loop() {

        int currentPos = turretMotor.getCurrentPosition();

        // ==========================
        // AUTO UNWIND MODE
        // ==========================
        if (unwinding) {

            int error = CENTER_POSITION - currentPos;

            if (Math.abs(error) < 10) {
                turretMotor.setPower(0);
                unwinding = false;
                resetPID();
                telemetry.addLine("Unwind Complete");
            } else {
                double power = UNWIND_POWER * Math.signum(error);
                turretMotor.setPower(power);
                telemetry.addLine("UNWINDING...");
            }

            telemetry.addData("Encoder", currentPos);
            telemetry.update();
            return;
        }

        // ==========================
        // CHECK LIMITS
        // ==========================
        if (currentPos >= MAX_TURRET_TICKS || currentPos <= MIN_TURRET_TICKS) {
            unwinding = true;
            telemetry.addLine("LIMIT REACHED - Starting Unwind");
            telemetry.update();
            return;
        }

        // ==========================
        // NORMAL VISION TRACKING
        // ==========================

        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

            if (fiducials != null && !fiducials.isEmpty()) {
                LLResultTypes.FiducialResult fiducial = fiducials.get(0);
                double tx = fiducial.getTargetXDegrees();

                double dt = timer.seconds();
                timer.reset();

                if (dt < 0.001) dt = 0.001;

                double error = tx;

                integral += error * dt;
                integral = Math.max(-50, Math.min(50, integral));

                double derivative = (error - lastError) / dt;

                double output =
                        (kP * error) + (kI * integral) + (kD * derivative);

                if (Math.abs(error) < POSITION_TOLERANCE) {
                    output = 0;
                    integral = 0;
                }

                if (output != 0 && Math.abs(output) < MIN_POWER) {
                    output = MIN_POWER * Math.signum(output);
                }

                output = Math.max(-MAX_POWER, Math.min(MAX_POWER, output));

                turretMotor.setPower(output);

                lastError = error;

                telemetry.addLine("TRACKING");
                telemetry.addData("TX", tx);
                telemetry.addData("Encoder", currentPos);
                telemetry.addData("Motor Power", output);

            } else {
                turretMotor.setPower(0);
                telemetry.addLine("No Tag Found");
            }

        } else {
            turretMotor.setPower(0);
            telemetry.addLine("No Valid Limelight Result");
        }

        telemetry.update();
    }

    private void resetPID() {
        integral = 0;
        lastError = 0;
    }

    @Override
    public void stop() {
        turretMotor.setPower(0);
        limelight.stop();
    }
}
