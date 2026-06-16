package org.firstinspires.ftc.teamcode.TeleOPs.oldRobot;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

@Disabled
@TeleOp (name = "!TeleOP2", group = "OldRobot")

public class FinishedTeleOP extends LinearOpMode{
    public enum Intake_Sensor {
        INTAKE,
        ROTATE,
        OUTAKE,
        FORCED_OUTTAKE
    }

    public static class Ball{
        public int pos = 0;
        public int color = 0;
    }
    Ball[] storage = {new Ball(), new Ball(), new Ball()};

    Limelight3A limelight;
    DcMotorEx turretMotor;
    Servo lift;
    DcMotor plate;
    DcMotor intake;
    DcMotorEx shooter;
    DistanceSensor distanceSensor = null;
    NormalizedColorSensor test_color;

    int plateTarget = 0;
    int ballsCollected = -1;
    double hue = 0;
    int i = 2;
    boolean GPPtoggle = false;
    boolean greenBallToggle = true;

    boolean movePlateRightToggle = false;
    boolean movePlateLeftToggle = false;

    boolean manualOuttakeToggle = false;
    boolean manualOuttakeOnce = false;

    boolean forcedEmpty = false;
    boolean forcedEmptyOnce = false;

    boolean forcedOuttakeToggle = false;

    boolean forceOpenServo = false;

    boolean full = false;  // Becomes true when plate is full
    boolean ballTaken = false;  // Acts as a toggle to switch between ROTATE and INTAKE states
    boolean shootFound = false;  // Becomes true when the sensor finds the correct ball (depending on color), only in OUTTAKE
    boolean ShooterReady = false;  // Becomes true when shooter motor reaches 1050 RPM, ideal RPM for shooting, in OUTTAKE and FORCED_OUTTAKE
    boolean servoOpen = false;  // Becomes true when the "gate" servo opens, it's also needed for the lift to go up
    boolean liftUp = false;  // Acts as a toggle so that the servo doesn't go up endlessly in OUTTAKE and FORCED_OUTTAKE, also needed to switch back to INTAKE state
    boolean servoFail = false;  // Acts as a toggle for gamepad2.y, in case ball gets stuck during OUTTAKE, the servo goes down
    boolean shot = false;  // Becomes true when servo goes back down in OUTTAKE and FORCED_OUTTAKE, also needed to empty the array element automatically and switch back to INTAKE
    boolean ballOut = false;  // Acts as a toggle to empty the array element in OUTTAKE and FORCED_OUTTAKE, becomes true when the ball is actually shot
    boolean outtakeDone = false;
    boolean PGPtoggle = false;
    boolean PPGtoggle = false;


    int pattern = 0;

    SampleMecanumDrive drive;
    Intake_Sensor state = Intake_Sensor.INTAKE;

    private double kP = 0.02;
    private double kI = 0.0001;
    private double kD = 0.002;

    private double integral = 0;
    private double lastError = 0;

    private ElapsedTime timer = new ElapsedTime();

    private static final int MAX_TURRET_TICKS = 50;
    private static final int MIN_TURRET_TICKS = -1600;

    private static final int CENTER_POSITION = 0;
    private static final double UNWIND_POWER = 1;

    private static final double POSITION_TOLERANCE = 1.5;
    private static final double MAX_POWER = 1;
    private static final double MIN_POWER = 0.05;

    private boolean unwinding = false;


    @Override
    public void runOpMode() throws InterruptedException {

        drive = new SampleMecanumDrive(hardwareMap);

        // In correlation with Configuratie.txt (HOPEFULLY)

        plate = hardwareMap.get(DcMotor.class, "plate");
        intake = hardwareMap.get(DcMotor.class, "intake");
        shooter = hardwareMap.get(DcMotorEx.class, "outtake");

        lift = hardwareMap.get(Servo.class, "lift");

        plate.setDirection(DcMotorSimple.Direction.REVERSE);

        distanceSensor = hardwareMap.get(DistanceSensor.class, "distance");
        test_color = hardwareMap.get(NormalizedColorSensor.class, "color");

        plate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        plate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        plate.setPower(1);
        plate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        plate.setTargetPosition(plateTarget);
        plate.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        double distance;
        NormalizedRGBA colors;

        while(opModeInInit() && !isStopRequested()){
            plate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            turretMotor = hardwareMap.get(DcMotorEx.class, "aim");

            turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.pipelineSwitch(1);
            limelight.start();

            timer.reset();

            telemetry.addLine("Turret Initialized - START CENTERED");
        }

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {

            int currentPos = turretMotor.getCurrentPosition();

            //unwind

            if (unwinding) {

                int error = CENTER_POSITION - currentPos;

                if (Math.abs(error) < 10) {
                    turretMotor.setPower(0);
                    unwinding = false;
                    integral = 0;
                    lastError = 0;
                    telemetry.addLine("Unwind Complete");
                } else {
                    double power = UNWIND_POWER * Math.signum(error);
                    turretMotor.setPower(power);
                    telemetry.addLine("UNWINDING...");
                }

                telemetry.addData("Encoder", currentPos);
                telemetry.update();
                continue;
            }

            // limits
            if (currentPos >= MAX_TURRET_TICKS || currentPos <= MIN_TURRET_TICKS) {
                unwinding = true;
                telemetry.addLine("LIMIT REACHED - Starting Unwind");
                telemetry.update();
                continue;
            }

            // normal tracking

            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {

                List<LLResultTypes.FiducialResult> fiducials =
                        result.getFiducialResults();

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




            // Turns RGB into hue for more sensor reliability
            colors = test_color.getNormalizedColors();

            double max = Math.max(colors.red, colors.blue);
            double min = Math.min(colors.red, colors.blue);

            max = Math.max(max, colors.green);
            min = Math.min(min, colors.green);

            double delta = max - min;

            if(delta == 0) hue = 0;

            if(max == colors.red) hue = 60 * (((colors.green - colors.blue) / delta) % 6);
            if(max == colors.green) hue = 60 * ((colors.blue - colors.red) / delta);
            if(max == colors.blue) hue = 60 * ((colors.red - colors.green) / delta);

            if(hue < 0) hue += 360;

            distance = distanceSensor.getDistance(DistanceUnit.MM);

            plate.setTargetPosition(plateTarget);

            drive.setWeightedDrivePower(new Pose2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x
            ));

            full = ballsCollected == 2;
            if(full) intake.setPower(0);

            if(ballsCollected < -1) ballsCollected = -1;
            if(ballsCollected > 2) ballsCollected = 2;


            switch(state){
                case INTAKE:
                    if(!forcedEmptyOnce) intake.setPower(1);

                    if(ballsCollected == -1){
                        plateTarget = 0;
                    }

                    if(distance < 100 && !full && plateTarget < plate.getCurrentPosition() + 5 && plateTarget > plate.getCurrentPosition() - 5 && !forcedEmptyOnce){
                        if(!ballTaken) {
                            ballsCollected += 1;
                            state = Intake_Sensor.ROTATE;
                            ballTaken = true;

                        } else ballTaken = false;
                    }

                    break;
                case ROTATE:

                    storage[ballsCollected].pos = plate.getCurrentPosition();
                    if(hue > 100) storage[ballsCollected].color = 2;
                    if(hue < 100) storage[ballsCollected].color = 1;

                    plateTarget += 472;
                    state = Intake_Sensor.INTAKE;

                    break;
                case OUTAKE:

                    intake.setPower(0);
                    shooter.setVelocity(1050);
                    if(shooter.getVelocity() > 1000){
                        ShooterReady = true;
                    }

                    if(pattern == 1 && !shootFound) {
                            for (int i = 2; i >= 0; i--) {
                                if (storage[i].color == 1) {
                                    plateTarget = storage[i].pos + 472;
                                    shootFound = true;
                                    pattern = 0;
                                    break;
                                }
                            }
                        }
                    if(pattern == 2 && !shootFound) {
                        for (int i = 2; i >= 0; i--) {
                            if (storage[i].color == 1) {
                                plateTarget = storage[i].pos + 472*3;
                                shootFound = true;
                                pattern = 0;
                                break;
                            }
                        }
                    }
                    if(pattern == 3 && !shootFound) {
                        for (int i = 2; i >= 0; i--) {
                            if (storage[i].color == 1) {
                                plateTarget = storage[i].pos + 472*2;
                                shootFound = true;
                                pattern = 0;
                                break;
                            }
                        }
                    }

                    if(shootFound && plate.getCurrentPosition() > plateTarget - 30 && plate.getCurrentPosition() < plateTarget + 30){
                        plate.setPower(0.5);
                        plateTarget = 0;
                    }

                    if(!outtakeDone && plate.getCurrentPosition() > plateTarget - 30 && plate.getCurrentPosition() < plateTarget + 30){
                        state = Intake_Sensor.INTAKE;
                        storage[0].pos = 0;
                        storage[1].pos = 0;
                        storage[2].pos = 0;
                        storage[0].color = 0;
                        storage[1].color = 0;
                        storage[2].color = 0;
                        ballsCollected = -1;

                    }
//
//
                    break;
                case FORCED_OUTTAKE:

                    intake.setPower(0);
                    shooter.setVelocity(1050);

                    servoOpen = false;

                    if(shooter.getVelocity() > 1000){
                        ShooterReady = true;
                    }

                    if(plate.getCurrentPosition() > plateTarget - 10 && plate.getCurrentPosition() < plateTarget + 10){
                        servoOpen = true;
                    }
                    if(servoOpen && ShooterReady && !liftUp && plate.getCurrentPosition() > plateTarget - 5 && plate.getCurrentPosition() < plateTarget + 5){
                        lift.setPosition(0.8);
                        liftUp = true;
                    }

                    if(ShooterReady && !shot){
                        lift.setPosition(0.46);
                        shot = true;
                    }

                    if(ShooterReady && shooter.getVelocity() < 1000 && !ballOut && shot){
                        ballsCollected -= 1;
                        storage[i].pos = 0;
                        storage[i].color = 0;
                        ballOut = true;
                    }

                    if(liftUp && shot) {
                        liftUp = false;
                        ShooterReady = false;
                        shootFound = false;
                        shot = false;
                        ballOut = false;
                        state = Intake_Sensor.INTAKE;
                    }
                    break;
            }


            if(gamepad2.dpad_right && ballsCollected > -1){
                if(!GPPtoggle)
                {
                    forcedEmptyOnce = true;
                    state = Intake_Sensor.OUTAKE;
                    pattern = 1;
                    GPPtoggle = true;
                }
            } else GPPtoggle = false;

            if(gamepad2.dpad_up && ballsCollected > -1){
                if(!PGPtoggle)
                {
                    forcedEmptyOnce = true;
                    state = Intake_Sensor.OUTAKE;
                    pattern = 2;
                    PGPtoggle = true;
                }
            } else PGPtoggle = false;

            if(gamepad2.dpad_right && ballsCollected > -1){
                if(!PPGtoggle)
                {
                    forcedEmptyOnce = true;
                    state = Intake_Sensor.OUTAKE;
                    pattern = 3;
                    PPGtoggle = true;
                }
            } else PPGtoggle = false;
//
//            if(gamepad2.left_bumper && ballsCollected > -1){  // Rotate plate until ball is green then shoot it
//                if(!greenBallToggle)
//                {
//                    forcedEmptyOnce = true;
//                    state = Intake_Sensor.OUTAKE;
//                    toShoot = 1;
//                    greenBallToggle = true;
//                }
//            } else greenBallToggle = false;



//            if(gamepad2.dpad_right && !movePlateRightToggle){  // Move plate to the right, only works when array is emptied
//                movePlateRightToggle = true;
//                desiredPlatePoz -= 178;
//            } else movePlateRightToggle = false;
//
//            if(gamepad2.dpad_left && !movePlateLeftToggle){  // Move plate to the left, only works when array is emptied
//                movePlateLeftToggle = true;
//                desiredPlatePoz += 178;
//            } else movePlateLeftToggle = false;



//            if(gamepad2.x){  // Forced outtake, used because regular outtake doesn't work after array is emptied
//                if(!forcedOuttakeToggle)
//                {
//                    forcedEmptyOnce = true;
//                    state = Intake_Sensor.FORCED_OUTTAKE;
//                    forcedOuttakeToggle = true;
//                }
//            } else forcedOuttakeToggle = false;
//
//
//
//            if(gamepad2.dpad_up){  // Slightly move plate left in case of bad alignment after autonomous (hold)
//                desiredPlatePoz += 2;
//            }
//            if(gamepad2.dpad_down){  // Slightly move plate right in case of bad alignment after autonomous (hold)
//                desiredPlatePoz -= 2;
//            }



            if(gamepad2.b){  // Toggle the outtake motor manually, outside of state machine if needed (so you don't have to wait until 1050 RPM at basket)
                if(!manualOuttakeToggle){
                    if(manualOuttakeOnce) shooter.setPower(0);
                    else shooter.setVelocity(1050);

                    manualOuttakeOnce = !manualOuttakeOnce;
                    manualOuttakeToggle = true;
                }
            } else manualOuttakeToggle = false;



            plate.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            telemetry.addData("====================:", "");
            telemetry.addData("State: ", state);
            telemetry.addData("Motor Speed: ", shooter.getVelocity());
            telemetry.addData("Ball:", storage[0].color);
            telemetry.addData("Ball:", storage[0].pos);
            telemetry.addData("Ball1:", storage[1].color);
            telemetry.addData("Ball1:", storage[1].pos);
            telemetry.addData("Ball2:", storage[2].color);
            telemetry.addData("Ball2:", storage[2].pos);
            telemetry.addData("platePos:", plateTarget);
            telemetry.addData("full:", full);
            telemetry.addData("balltaken :", ballTaken);
            telemetry.addData("====================:", "");
            telemetry.addData("Distance: ", distanceSensor.getDistance(DistanceUnit.MM));
            telemetry.addData("hue: ", hue);
            telemetry.addData("BallsCollected:", ballsCollected);
            telemetry.addData("plateTarget:", plateTarget);
            telemetry.addData("currentPlatePoz:" , plate.getCurrentPosition());

            telemetry.update();
        }
    }
}