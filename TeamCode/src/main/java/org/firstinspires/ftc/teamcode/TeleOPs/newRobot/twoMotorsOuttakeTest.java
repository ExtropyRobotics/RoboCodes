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
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.Encoder;

import java.util.List;

@Disabled
@TeleOp(name = "!2motorOuttake", group = "Tester")
public class twoMotorsOuttakeTest extends LinearOpMode {

    public enum StateList {
        Intake,
        Rotate,
        Outtake
    }
    StateList state = StateList.Intake;
    public static class Artefact{
        int pos = 0;
        int color = 0;
    }

    Artefact[] storage = {new Artefact(), new Artefact(), new Artefact()};


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
    double maxPlatePower = 1;

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
    private boolean tagVisible = true;

    // hardware
    Encoder plateEncoder;
    CRServo plateServoLeft;
    CRServo plateServoRight;
    DcMotorEx intake;
    DcMotorEx outtake;
    DcMotorEx outtake2;
    NormalizedColorSensor colorSensor;
    DistanceSensor distanceSensor;
    IMU imu;

    int plateTargetPosition = 0;
    int i = 2;
    int pattern = 0;

    int sumPlate = 0;
    int encoderPos = 0;
    double plateError = 0;

    double platePow = 1;
    double plateP = 0.0002;
    double plateI = 0.00000;
    double plateD = 0.0008;

    double artefactDistance = 0;
    NormalizedRGBA colors;

    ElapsedTime i2cTimer = new ElapsedTime();

    int artefactsGathered = -1;
    boolean full = false;
    boolean plateAtPosition = false;
    boolean artefactTaken = false;
    boolean shootFound = false;
    boolean shooterReady = false;
    boolean atPattern = false;

    boolean pattern1Toggle = false;
    boolean pattern2Toggle = false;
    boolean pattern3Toggle = false;
    boolean manualPlateMove = false;
    boolean ballOneShot = false;
    boolean ballTwoShot = false;
    boolean ballThreeShot = false;

    ElapsedTime outtakeStop = new ElapsedTime();
    ElapsedTime shootCooldown = new ElapsedTime();

    @Override
    public void runOpMode() {

        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);

        drive = new SampleMecanumDrive(hardwareMap);

        plateServoLeft = hardwareMap.get(CRServo.class, "plateServoLeft");
        plateServoRight = hardwareMap.get(CRServo.class, "plateServoRight");

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        outtake = hardwareMap.get(DcMotorEx.class, "outtake");
        turret = hardwareMap.get(DcMotorEx.class, "turret");

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");

        imu = hardwareMap.get(IMU.class, "imu");

        plateEncoder = hardwareMap.get(Encoder.class, "outtake2");
        plateEncoder.setDirection(Encoder.Direction.REVERSE);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        outtake2 = hardwareMap.get(DcMotorEx.class, "outtake2");

        outtake.setDirection(DcMotorSimple.Direction.REVERSE);

        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        );

        // TODO: Set your AprilTag pipeline here before each match
        // 1 - Blue alliance
        // 2 - Red alliance
        limelight.pipelineSwitch(1);
        limelight.start();

        i2cTimer.reset();

        waitForStart();
        double dt = timer.seconds();

        timer.reset();
        outtakeStop.reset();

        while (opModeIsActive() && !isStopRequested()) {

            dt = timer.seconds();

            plateAtPosition = plateTargetPosition < plateEncoder.getCurrentPosition() + 200 && plateTargetPosition > plateEncoder.getCurrentPosition() - 200;

            double plateDt = timer.seconds();

            encoderPos = plateEncoder.getCurrentPosition();
            plateError = plateTargetPosition - encoderPos;
            sumPlate += (int) (plateError * plateDt);
            double deriv = (plateTargetPosition - encoderPos - plateError);

            platePow = plateP*plateError + plateI*sumPlate + deriv* plateD;

            movementStart();
            turretStart();
            imuStart();

            intake.setPower(1);

            if(i2cTimer.seconds() >= 0.1){
                artefactDistance = distanceSensor.getDistance(DistanceUnit.MM);
                colors = colorSensor.getNormalizedColors();
                i2cTimer.reset();
            }

            if(artefactsGathered == 2) full = true;

            switch (state){
                case Intake:

                    if(artefactsGathered == -1) plateTargetPosition = 0;

                    if(plateAtPosition){
                        if(artefactDistance < 100 && !full){
                            if(!artefactTaken){
                                artefactsGathered += 1;
                                state = StateList.Rotate;
                                artefactTaken = true;
                            } else artefactTaken = false;
                        }
                    }

                    if(outtakeStop.seconds() > 5){
                        outtake.setPower(0);
                        outtake2.setPower(0);
                    }

                    shootCooldown.reset();

                    break;

                case Rotate:

                    storage[artefactsGathered].pos = plateEncoder.getCurrentPosition();

                    if(getHue() > 100) storage[artefactsGathered].color = 2;
                    if(getHue() < 100) storage[artefactsGathered].color = 1;

                    plateTargetPosition += 8192/3;
                    state = StateList.Intake;

                    break;

                case Outtake:
                    outtake.setPower(1);
                    outtake2.setPower(1);

                    if(outtake.getVelocity() > 1700) {
                        shooterReady = true;
                    } else shooterReady = false;

                    if (!shootFound) {
                        for (i = 0; i <= 2; i++) {
                            if (storage[i].color == 1) {
                                plateTargetPosition += storage[i].pos/3 + 3000;
                                shootFound = true;
                                break;
                            }
                        }
                    }

                    if(shootFound && !atPattern){
                        if(pattern == 3) plateTargetPosition += 2*8192/3;
                        if(pattern == 2) plateTargetPosition += 8192/3;

                        atPattern = true;
                        pattern = 0;
                    }

                    if(shooterReady && shootFound && plateAtPosition && atPattern && !ballOneShot) {
                        plateTargetPosition -= 8192/3;

                        storage[0].pos = 0;
                        storage[0].color = 0;
                        shootCooldown.reset();
                        ballOneShot = true;
                        full = false;
                    }

                    if(ballOneShot && !ballTwoShot && shootCooldown.seconds() >= 1){
                        plateTargetPosition -= 8192/3;

                        storage[1].pos = 0;
                        storage[1].color = 0;

                        shootCooldown.reset();
                        ballTwoShot = true;
                    }

                    if(ballTwoShot && !ballThreeShot && shootCooldown.seconds() >= 1){
                        plateTargetPosition -= 8192/3;

                        storage[2].pos = 0;
                        storage[2].color = 0;
                        ballThreeShot = true;
                        artefactsGathered = -1;
                    }

                    if(ballThreeShot){
                        state = StateList.Intake;
                        ballOneShot = false;
                        ballTwoShot = false;
                        ballThreeShot = false;
                        shooterReady = false;
                        shootFound = false;
                        pattern = 0;
                        atPattern = false;
                        outtakeStop.reset();
                    }

                    break;
            }

            if(gamepad2.x && !pattern1Toggle){
                pattern = 1;
                state = StateList.Outtake;
                pattern1Toggle = true;
            } else pattern1Toggle = false;

            if(gamepad2.y && !pattern2Toggle){
                pattern = 2;
                state = StateList.Outtake;
                pattern2Toggle = true;
            } else pattern2Toggle = false;

            if(gamepad2.b && !pattern3Toggle){
                pattern = 3;
                state = StateList.Outtake;
                pattern3Toggle = true;
            } else pattern3Toggle = false;

            if(gamepad2.dpad_left && !manualPlateMove){
                plateTargetPosition += 8129/3;
                manualPlateMove = true;
            } else manualPlateMove = false;

            if(platePow < -maxPlatePower) platePow = -maxPlatePower;
            if(platePow > maxPlatePower) platePow = maxPlatePower;

            plateServoRight.setPower(platePow);
            plateServoLeft.setPower(platePow);

            telemetry.addData("state: ", state);
            telemetry.addData("Ball 1 color: ", storage[0].color);
            telemetry.addData("Ball 1 pos: ", storage[0].pos);
            telemetry.addData("Ball 2 color: ", storage[1].color);
            telemetry.addData("Ball 2 pos: ", storage[1].pos);
            telemetry.addData("Ball 3 color:", storage[2].color);
            telemetry.addData("Ball 3 pos:", storage[2].pos);
            telemetry.addData("outtake RPM: ", outtake.getVelocity());
            telemetry.addData("plateAtPosition: ", plateAtPosition);
            telemetry.addData("platePos: ", plateEncoder.getCurrentPosition());
            telemetry.addData("plateTarget: ", plateTargetPosition);
            telemetry.addData("artefactDistance: ", artefactDistance);
            telemetry.addData("hue: ", getHue());
            telemetry.update();
        }
    }

    public void movementStart(){
        drive.setWeightedDrivePower(new Pose2d(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x
        ));
    }
    public void imuStart(){
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());
        LLResult llResult = limelight.getLatestResult();
        if(llResult != null && llResult.isValid()){
            Pose3D botPose = llResult.getBotpose_MT2();
        }
    }
    public void turretStart() {
        double dt = timer.seconds();
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
            return;
        }

        if (currentPos >= MAX_TURRET_TICKS || currentPos <= MIN_TURRET_TICKS) {
            unwinding = true;
            return;
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
    }

    public double getHue(){
        double hue = 0;
        double max = Math.max(colors.red, colors.blue);
        double min = Math.min(colors.red, colors.blue);

        max = Math.max(max, colors.green);
        min = Math.min(min, colors.green);

        double delta = max - min;

        if (delta == 0) hue = 0;

        if (max == colors.red) hue = 60 * (((colors.green - colors.blue) / delta) % 6);
        if (max == colors.green) hue = 60 * ((colors.blue - colors.red) / delta);
        if (max == colors.blue) hue = 60 * ((colors.red - colors.green) / delta);

        if (hue < 0) hue += 360;
        return hue;
    }
}
