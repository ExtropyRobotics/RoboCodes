package org.firstinspires.ftc.teamcode.TeleOPs.newRobot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.List;

@Disabled
@TeleOp(name = "!caiet")
public class caiet extends LinearOpMode {

    // States
    public enum StateList{
        Intake,
        Rotate,
        Outtake
    }
    StateList state = StateList.Intake;
    StateList previousState = null;

    // Storage system
    public static class Artefact{
        public double pos = 0;
        public double color = 0;
    }

    Artefact[] storage = {new Artefact(), new Artefact(), new Artefact()};

    // TODO: tune the turret
    // PID default values for tuning turret
    private static final double kPturret = 0.02;
    private static final double kIturret = 0.0;
    private static final double kDturret = 0.0;

    private double integralTurret = 0;
    private double lastErrorTurret = 0;

    // Predictive tracking
    private double lastTxturret = 0;
    private static final double lookaheadTimeTurret = 0.10;

    private static final ElapsedTime timer = new ElapsedTime();

    // Turret limits
    private static final int MAX_TURRET_TICKS = 750;
    private static final int MIN_TURRET_TICKS = -1460;

    private static final double UNWIND_POWER = 0.5;

    // Tuning defaults
    private static final double POSITION_TOLERANCE = 1.5;
    private static final double MAX_TURRET_POWER = 1.0;
    private static final double MIN_TURRET_POWER = 0.05;

    private boolean unwinding = false;
    private int unwindDirection = 0;

    // Flywheel values
    private static final double highVelocity = 1100;
    private static final double lowVelocity = 950;
    private static final double curTargetVelocity = highVelocity;

    private static final double flywheelF = 0;
    private static final double flywheelP = 0;

    // Sensor attributes
    double artefactDistance = 0;
    NormalizedRGBA colors = null;
    double hue = 0;

    // Encode attributes
    double leftPose = 0;
    double rightPose = 0;

    // State machine logic variables
    int i = 1;
    double artefactsGathered = 0;
    boolean full = false;
    boolean plateAtTarget = false;
    boolean artefactTaken = false; // A toggle for when the intake state swithces to rotate
    boolean shooterReady = false; // True when motor reaches optimal RPM for shooting
    boolean shootFound = false; // True when the color sensor detects the artefact with the coresponding color
    int pattern = 0; // 1 for GPP, 2 for PGP, 3 for PPG
    boolean outtakeCompleted = false;

    // Plate PID
    private double plateTarget = 0;
    private static final double plateKp = 0.6;   // example, tune in teleop
    private static final double plateKi = 0;
    private static final double plateKd = 0;
    private double plateIntegral = 0;
    private double plateLastError = 0;
    private static final double PLATE_POSITION_TOLERANCE = 0.03; // fraction of full rotation
    private double currentPlatePos = getPlatePosition();

    // Ramp variables
    private static final double scale = 2.178367;
    // Timers
    ElapsedTime sensorCooldown = new ElapsedTime();

    // Hardware
    Servo ramp;
    CRServo plateServoLeft;
    CRServo plateServoRight;
    DcMotorEx intake;
    DcMotorEx outtake;
    DcMotorEx turret;

    AnalogInput leftEncoder;
    AnalogInput rightEncoder;

    NormalizedColorSensor colorSensor;
    DistanceSensor distanceSensor;

    IMU imu;

    Limelight3A limelight;

    SampleMecanumDrive drive;

    @Override
    public void runOpMode() {

        // Configuration
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);

        drive = new SampleMecanumDrive(hardwareMap);

        plateServoLeft = hardwareMap.get(CRServo.class, "plateServoLeft");
        plateServoRight = hardwareMap.get(CRServo.class, "plateServoRight");

        leftEncoder = hardwareMap.get(AnalogInput.class, "leftEncoder");
        rightEncoder = hardwareMap.get(AnalogInput.class, "rightEncoder");

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        outtake = hardwareMap.get(DcMotorEx.class, "outtake");
        turret = hardwareMap.get(DcMotorEx.class, "turret");

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");

        ramp = hardwareMap.get(Servo.class, "ramp");

        imu = hardwareMap.get(IMU.class, "imu");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP);

        // TODO: Set your custom AprilTag pipeline here before each match
        // 1 - Blue alliance
        // 2 - Red alliance
        limelight.pipelineSwitch(1);
        limelight.start();

        waitForStart();

        // Resetting timers
        timer.reset();
        sensorCooldown.reset();

        while (opModeIsActive() && !isStopRequested()) {
            // Initialize basic functions
            turretStart();
            getGoalDistance();
            sensorStart();
            plateCalculations();

            // Helps reduce lag
            // i2c devices detecting every loop causes lag, so a cooldown is added
            if(sensorCooldown.seconds() >= 0.1){
                artefactDistance = distanceSensor.getDistance(DistanceUnit.MM);
                colors = colorSensor.getNormalizedColors();
                sensorCooldown.reset();
            }

            // Logic variables in use
            if(artefactsGathered == 3) full = true;

            if(full) intake.setPower(0);

            // Variable limits
            if(artefactsGathered < 0) artefactsGathered = 0;
            if(artefactsGathered > 3) artefactsGathered = 3;

            // Clamp plateTarget between 0-1 (one full rotation)
            if(plateTarget > 1) plateTarget -= 1;
            if(plateTarget < 0) plateTarget += 1;

            // Automatically switch ramp positions based on distance to goal
            if(getGoalDistance() >= 0 && getGoalDistance() <= 30) ramp.setPosition(0.2);
            if(getGoalDistance() > 30 && getGoalDistance() <= 80) ramp.setPosition(0.3);
            if(getGoalDistance() > 80 && getGoalDistance() <= 130) ramp.setPosition(0.4);
            if(getGoalDistance() > 130 && getGoalDistance() <= 180) ramp.setPosition(0.5);
            if(getGoalDistance() > 180 && getGoalDistance() <= 230) ramp.setPosition(0.6);
            if(getGoalDistance() > 230 && getGoalDistance() <= 280) ramp.setPosition(0.7);

            // State machine
            switch (state){

                case Intake:
                    // Start intake motor
                    if(!full) intake.setPower(1);

                    // Stop outtake motor
                    outtake.setVelocity(0);

                    if(plateAtTarget){
                        // When an artefact is detected, switch states
                        if(artefactDistance < 100 && !full){
                            if(!artefactTaken){
                                artefactsGathered += 1;
                                state = StateList.Rotate;
                                artefactTaken = true;
                            } else artefactTaken = false;

                        }
                    }

                    previousState = state;
                    break;

                case Rotate:
                    // Store artefact values regarding plate slot position
                    storage[(int) artefactsGathered].pos = getPlatePosition();

                    if(getHue() > 100) storage[(int) artefactsGathered].color = 2;
                    if(getHue() < 100) storage[(int) artefactsGathered].color = 1;

                    // Rotate plate
                    plateTarget += 0.333;
                    state = StateList.Intake;

                    previousState = state;
                    break;

                case Outtake:
                    // Stop intake motor
                    intake.setPower(0);

                    // Start outtake motor
                    outtake.setVelocity(curTargetVelocity);

                    // Wait for motor to reach optimal RPM
                    if(outtake.getVelocity() > 1000) shooterReady = true;
                    else shooterReady = false;

                    // Rotate plate until correct artefact pattern regarding the color is found
                    if (!shootFound) {
                            for (i = 1; i <= 3; i++) {
                                if (storage[i].color == 1) {
                                    plateTarget = storage[i].pos + 0.666;
                                    if(pattern == 1) plateTarget += 0;
                                    if(pattern == 2) plateTarget += 0.333;
                                    if(pattern == 3) plateTarget += 0.666;
                                    shootFound = true;
                                    break;
                                }
                            }
                        }

                    // Rotate plate backwards, to initial position, to launch the artefacts
                    if(shooterReady && shootFound && !outtakeCompleted && plateAtTarget){
                        plateTarget = 0;
                        outtakeCompleted = true;

                        // Empty the array
                        storage[0].pos = 0;
                        storage[2].pos = 0;
                        storage[1].pos = 0;

                        storage[0].color = 0;
                        storage[1].color = 0;
                        storage[2].color = 0;

                        artefactsGathered = 0;
                        full = false;
                    }

                    // Go back into intake and reset every used variable
                    if(outtakeCompleted){
                        state = StateList.Intake;
                        shooterReady = false;
                        shootFound = false;
                        outtakeCompleted = false;
                        pattern = 0;
                    }

                    previousState = state;
                    break;
            }

            // ==== BUTTONS ====

            movementStart(); // Gamepad1 sticks for movement

            // Plate manual buttons
            if(gamepad1.dpad_right) plateTarget += 0.333; // next slot
            if(gamepad1.dpad_left) plateTarget -= 0.333;  // previous slot
            if(gamepad1.dpad_up) plateTarget += 0.05;    // fine adjust up
            if(gamepad1.dpad_down) plateTarget -= 0.05;  // fine adjust down

            // Shooting

            if(gamepad2.xWasPressed()){ // Shoot GGP
                pattern = 1;
                state = StateList.Outtake;
            }
            if(gamepad2.yWasPressed()){ // Shoot PGP
                pattern = 2;
                state = StateList.Outtake;
            }
            if(gamepad2.bWasPressed()){ // Shoot PPG
                pattern = 3;
                state = StateList.Outtake;
            }

            // Start outtake manually so you don't have to wait in outtake state
            if(gamepad2.aWasPressed()) outtake.setVelocity(currentPlatePos);
        }
    }

    public double getHue(){
        // Calculating hue value
        if(colors != null) {
            double max = Math.max(colors.red, colors.blue);
            double min = Math.min(colors.red, colors.blue);

            max = Math.max(max, colors.green);
            min = Math.min(min, colors.green);

            double delta = max - min;

            if (delta == 0) hue = 0;

            if (max == colors.red) hue = 60 *
                    (((colors.green - colors.blue) / delta) % 6);
            if (max == colors.green) hue = 60 *
                    ((colors.blue - colors.red) / delta);
            if (max == colors.blue) hue = 60 *
                    ((colors.red - colors.green) / delta);

            if (hue < 0) hue += 360;
        }
        return hue;
    }

    public void sensorStart(){
        // Initializing plate encoders
        leftPose = 1 - leftEncoder.getVoltage() / 3.3;
        rightPose = rightEncoder.getVoltage() / 3.3; // polar opposite
    }

    public void movementStart(){
        // Attributing movement
        drive.setWeightedDrivePower(new Pose2d(
                gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                gamepad1.right_stick_x
        ));
    }

    public double getGoalDistance(){
        // Calculating the distance to the goal aprilTag
        double ta = 0;
        double goalDistance = 0;

        LLResult llResult = limelight.getLatestResult();
        if(llResult != null && llResult.isValid()){
            ta = llResult.getTa();
        }

        goalDistance = (scale / ta);
        return goalDistance;
    }

    public void turretStart() {
        int currentPos = turret.getCurrentPosition();

        if (unwinding) {
            turret.setPower(unwindDirection * UNWIND_POWER);
            if (currentPos < MAX_TURRET_TICKS - 100 && currentPos > MIN_TURRET_TICKS + 100) {
                unwinding = false;
                integralTurret = 0;
                lastErrorTurret = 0;
            }
            return;
        }

        if (currentPos >= MAX_TURRET_TICKS) {
            unwinding = true;
            unwindDirection = -1;
            return;
        }

        if (currentPos <= MIN_TURRET_TICKS) {
            unwinding = true;
            unwindDirection = 1;
            return;
        }

        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            if (fiducials != null && !fiducials.isEmpty()) {
                LLResultTypes.FiducialResult tag = fiducials.get(0);
                double tx = tag.getTargetXDegrees();
                double dt = timer.seconds();
                timer.reset();
                if (dt < 0.001) dt = 0.001;

                double velocity = (tx - lastTxturret) / dt;
                double predictedTx = tx + velocity * lookaheadTimeTurret;
                predictedTx = Math.max(-30, Math.min(30, predictedTx));

                double error = predictedTx;

                if (Math.abs(error) < POSITION_TOLERANCE) {
                    turret.setPower(0);
                    integralTurret = 0;
                    lastErrorTurret = error;
                    lastTxturret = tx;
                    return;
                }

                integralTurret += error * dt;
                integralTurret = Math.max(-40, Math.min(40, integralTurret));
                double derivative = (error - lastErrorTurret) / dt;

                double output = (kPturret * error) + (kIturret * integralTurret) + (kDturret * derivative);

                if (Math.abs(output) < MIN_TURRET_POWER) {
                    output = MIN_TURRET_POWER * Math.signum(output);
                }

                output = Math.max(-MAX_TURRET_POWER, Math.min(MAX_TURRET_POWER, output));

                turret.setPower(output);

                lastErrorTurret = error;
                lastTxturret = tx;
            } else {
                turret.setPower(0);
            }
        } else {
            turret.setPower(0);
        }
    }

    public double getPlatePosition(){
        // Average encoder method
        return (leftPose + (1 - rightPose)) / 2.0;
    }

    public void plateCalculations(){
        // Plate PID
        double plateError = plateTarget - currentPlatePos;

        // Wrap error around 0-1 for shortest path
        if(plateError > 0.5) plateError -= 1;
        if(plateError < -0.5) plateError += 1;

        plateIntegral += plateError * 0.02; // loop dt ~20ms
        double plateDerivative = (plateError - plateLastError) / 0.02;

        double plateOutput = plateKp * plateError + plateKi * plateIntegral + plateKd * plateDerivative;

        // minimum output to overcome servo friction
        if(Math.abs(plateOutput) < 0.05) plateOutput = 0.05 * Math.signum(plateOutput);

        // Clamp
        plateOutput = Math.max(-1.0, Math.min(1.0, plateOutput));

        plateServoLeft.setPower(plateOutput);
        plateServoRight.setPower(-plateOutput); // polar opposite

        // plateAtTarget boolean
        plateAtTarget = Math.abs(plateError) < PLATE_POSITION_TOLERANCE;
    }
}