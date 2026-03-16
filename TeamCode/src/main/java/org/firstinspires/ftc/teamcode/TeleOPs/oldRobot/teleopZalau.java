package org.firstinspires.ftc.teamcode.TeleOPs.oldRobot;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.AnalogInput;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

@TeleOp (name = "!TeleopZALAU", group = "OldRobot")

public class teleopZalau extends LinearOpMode{
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
    DcMotor plate;
    DcMotor intake;
    DcMotorEx shooter;
    Servo lift;
    DistanceSensor distanceSensor = null;
    NormalizedColorSensor test_color;

    int plateTarget = 0;
    int desiredPlatePoz = 0;
    int ballsCollected = -1;
    int i = 2;
    int toShoot = 0;
    int outtakeCounter = 0;

    double hue = 0;

    boolean outtakeToggle = false;
    boolean intakeToggleButton = true;

    boolean movePlateRightToggle = false;
    boolean movePlateLeftToggle = false;

    boolean manualOuttakeToggle = false;
    boolean manualOuttakeOnce = false;

    boolean forcedEmpty = false;
    boolean forcedEmptyOnce = false;

    boolean forcedOuttakeToggle = false;

    boolean full = false;  // Becomes true when plate is full
    boolean ballTaken = false;  // Acts as a toggle to switch between ROTATE and INTAKE states
    boolean shootFound = false;  // Becomes true when the sensor finds the correct ball (depending on color), only in OUTTAKE
    boolean ShooterReady = false;  // Becomes true when shooter motor reaches 1050 RPM, ideal RPM for shooting, in OUTTAKE and FORCED_OUTTAKE
    boolean servoOpen = false;  // Becomes true when the "gate" servo opens, it's also needed for the lift to go up
    boolean liftUp = false;  // Acts as a toggle so that the servo doesn't go up endlessly in OUTTAKE and FORCED_OUTTAKE, also needed to switch back to INTAKE state
    boolean servoFail = false;  // Acts as a toggle for gamepad2.y, in case ball gets stuck during OUTTAKE, the servo goes down
    boolean shot = false;  // Becomes true when servo goes back down in OUTTAKE and FORCED_OUTTAKE, also needed to empty the array element automatically and switch back to INTAKE
    boolean ballOut = false;  // Acts as a toggle to empty the array element in OUTTAKE and FORCED_OUTTAKE, becomes true when the ball is actually shot
    boolean intakeToggle = false;
    boolean intakeOnce = false;
    boolean intakeButtonToggle = false;
    boolean outtakeButtonToggle = false;

    double outtakeRPM = 0;
    double rateOfChange = 0;
    double oldVelocity = 0;

    SampleMecanumDrive drive;
    Intake_Sensor state = Intake_Sensor.INTAKE;
    Intake_Sensor prevState = null;
    ElapsedTime colorWait = new ElapsedTime();
    ElapsedTime shooterWait = new ElapsedTime();
    ElapsedTime servoWait = new ElapsedTime();
    ElapsedTime i2cTimer = new ElapsedTime();

    private double kP = 0.03;
    private double kI = 0.0001;
    private double kD = 0.002;

    private double integral = 0;
    private double lastError = 0;

    private ElapsedTime timer = new ElapsedTime();

    private static final int MAX_TURRET_TICKS = 750;
    private static final int MIN_TURRET_TICKS = -1460;

    private static final int CENTER_POSITION = 0;
    private static final double UNWIND_POWER = 1;

    private static final double POSITION_TOLERANCE = 1.5;
    private static final double MAX_POWER = 1;
    private static final double MIN_POWER = 0.05;

    private boolean unwinding = false;
    AnalogInput encoderLift;

    @Override
    public void runOpMode() throws InterruptedException {

        drive = new SampleMecanumDrive(hardwareMap);

        // In correlation with Configuratie.txt (HOPEFULLY)

        plate = hardwareMap.get(DcMotor.class, "plate");
        intake = hardwareMap.get(DcMotor.class, "intake");
        shooter = hardwareMap.get(DcMotorEx.class, "outtake");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        lift = hardwareMap.get(Servo.class, "lift");

        turretMotor = hardwareMap.get(DcMotorEx.class, "aim");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distance");
        test_color = hardwareMap.get(NormalizedColorSensor.class, "color");

        encoderLift = hardwareMap.get(AnalogInput.class, "lift_encoder");

        plate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        plate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        plate.setPower(1);
        plate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        plate.setTargetPosition(plateTarget);
        plate.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        double distance = 200;
        NormalizedRGBA colors = null;

        double liftPoz;

        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        limelight.pipelineSwitch(1);
        limelight.start();

        timer.reset();

        shooterWait.reset();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {

            if (shooterWait.seconds() > 0.1) {
                rateOfChange = shooter.getVelocity() - oldVelocity;
                oldVelocity = shooter.getVelocity();
                shooterWait.reset();
            }

            int currentPos = turretMotor.getCurrentPosition();

            //unwind
            if (unwinding) {
                int error = CENTER_POSITION - currentPos;

                if (Math.abs(error) < 10) {
                    turretMotor.setPower(0);
                    unwinding = false;
                    integral = 0;
                    lastError = 0;
                } else {
                    double power = UNWIND_POWER * Math.signum(error);
                    turretMotor.setPower(power);
                }
                continue;
            }

            // limits
            if (currentPos >= MAX_TURRET_TICKS || currentPos <= MIN_TURRET_TICKS) {
                unwinding = true;
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

            if(i2cTimer.seconds() >= 0.1){
                distance = distanceSensor.getDistance(DistanceUnit.MM);
                colors = test_color.getNormalizedColors();
                i2cTimer.reset();
            }


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

            liftPoz = 1 - encoderLift.getVoltage() / 3.3;


            plate.setTargetPosition(plateTarget);

            outtakeRPM = shooter.getVelocity();

            drive.setWeightedDrivePower(new Pose2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x
            ));

            full = ballsCollected == 2;
            if (full) intake.setPower(0);

            if (ballsCollected < -1) ballsCollected = -1;
            if (ballsCollected > 2) ballsCollected = 2;

            if (i == -1) i = 2;
            if (i == 3) i = 0;


            switch (state) {
                case INTAKE:
                    servoWait.reset();
                    intake.setPower(1);
                    lift.setPosition(0.55);
                    if (!forcedEmptyOnce) intake.setPower(1);
                    if(prevState == Intake_Sensor.OUTAKE){
                        plateTarget -= 237;
                    }
                    if(prevState == Intake_Sensor.INTAKE && plateTarget < plate.getCurrentPosition() + 5 && plateTarget > plate.getCurrentPosition() - 5){

                        if (ballsCollected == -1) {
                            plateTarget = desiredPlatePoz;
                        }

                        if (distance < 130 && !full && plateTarget < plate.getCurrentPosition() + 5 && plateTarget > plate.getCurrentPosition() - 5) {
                            if (!ballTaken) {
                                ballsCollected += 1;
                                state = Intake_Sensor.ROTATE;
                                ballTaken = true;

                            } else ballTaken = false;
                        }
                    }
                    prevState = state;
                    break;
                case ROTATE:
                    prevState = state;

                    storage[ballsCollected].pos = plate.getCurrentPosition();
                    if (hue > 100) storage[ballsCollected].color = 2;
                    if (hue < 100) storage[ballsCollected].color = 1;

                    plateTarget += 475;
                    state = Intake_Sensor.INTAKE;

                    break;
                case OUTAKE:

                    if(prevState == Intake_Sensor.INTAKE){
                        plateTarget += 237;
                        outtakeCounter = 0;
                    }

                    if (prevState == Intake_Sensor.OUTAKE && plateTarget < plate.getCurrentPosition() + 5 && plateTarget > plate.getCurrentPosition() - 5) {
                        intake.setPower(0);
                        shooter.setVelocity(1250);

                        if (outtakeRPM > 1200 && outtakeRPM < 1300 && rateOfChange < 40) {
                            ShooterReady = true;
                        }

//                        if (!shootFound) {
//                            for (i = 2; i >= 0; i--) {
//                                if (storage[i].color == toShoot) {
//                                    plateTarget = storage[i].pos + 475;
//                                    shootFound = true;
//                                    break;
//                                }
//                            }
//                        }

                        if (ShooterReady && liftPoz < 0.6 && !liftUp && plate.getCurrentPosition() > plateTarget - 5 && plate.getCurrentPosition() < plateTarget + 5) {
                            lift.setPosition(1);
                            liftUp = true;
                            servoWait.reset();
                        }

                        if (liftPoz > 0.7 && ShooterReady && !shot && liftUp) {
                            lift.setPosition(0.55);
                            shot = true;
                        }

                        if (ShooterReady && outtakeRPM > 1100 && !ballOut && shot) {
                            ballsCollected -= 1;
                            storage[i].pos = 0;
                            storage[i].color = 0;
                            ballOut = true;
                        }

                        if (liftPoz < 0.6 && liftUp && shot) {
                            liftUp = false;
                            ShooterReady = false;
                            shootFound = false;
                            shot = false;
                            ballOut = false;
                            plateTarget += 475;
                            outtakeCounter += 1;
                        }
                    }
                    prevState = state;
                    if(outtakeCounter == 3) state = Intake_Sensor.INTAKE;
                    break;
                case FORCED_OUTTAKE:

                    intake.setPower(0);
                    shooter.setVelocity(1250);

                    if (outtakeRPM > 1200 && outtakeRPM < 1300 && rateOfChange < 40) {
                        ShooterReady = true;
                    }

                    if (!shootFound) {
                        plateTarget += 237;
                        shootFound = true;
                    }

                    if (ShooterReady && liftPoz < 0.6 && !liftUp && plate.getCurrentPosition() > plateTarget - 5 && plate.getCurrentPosition() < plateTarget + 5) {
                        lift.setPosition(1);
                        liftUp = true;
                        servoWait.reset();
                    }

                    if (liftPoz > 0.7 && ShooterReady && !shot && liftUp) {
                        lift.setPosition(0.55);
                        shot = true;
                    }

                    if (ShooterReady && outtakeRPM > 1100 && !ballOut && shot) {
                        ballsCollected -= 1;
                        storage[i].pos = 0;
                        storage[i].color = 0;
                        ballOut = true;
                    }

                    if (liftPoz < 0.6 && liftUp && shot) {
                        liftUp = false;
                        ShooterReady = false;
                        shootFound = false;
                        shot = false;
                        ballOut = false;
                        plateTarget -= 237;
                        state = Intake_Sensor.INTAKE;
                    }
                    break;
            }
                    if (gamepad2.a && !forcedEmpty) {  // Empties the array, in case of wrong color detected, also makes it possible to move plate using dpads
                        forcedEmptyOnce = false;
                        state = Intake_Sensor.INTAKE;
                        forcedEmpty = true;
                        storage[0].pos = 0;
                        storage[1].pos = 0;
                        storage[2].pos = 0;
                        storage[0].color = 0;
                        storage[1].color = 0;
                        storage[2].color = 0;
                        ballsCollected = -1;
                    } else forcedEmpty = false;

                    if(gamepad1.b && !intakeButtonToggle){
                        state = Intake_Sensor.INTAKE;
                        intakeButtonToggle = true;
                    } else intakeButtonToggle = false;

                    if(gamepad1.x && !outtakeButtonToggle){
                        state = Intake_Sensor.OUTAKE;
                        outtakeButtonToggle = true;
                    } else outtakeButtonToggle = false;

                    if (gamepad2.right_bumper) {  // Rotate plate until ball is purple then shoot it
                        if (!outtakeToggle) {
                            forcedEmptyOnce = true;
                            state = Intake_Sensor.INTAKE;
                            outtakeToggle = true;
                        }
                    } else outtakeToggle = false;

                    if (gamepad2.left_bumper) {  // Rotate plate until ball is green then shoot it
                        if (!intakeToggleButton) {
                            forcedEmptyOnce = true;
                            state = Intake_Sensor.OUTAKE;
                            intakeToggleButton = true;
                        }
                    } else intakeToggleButton = false;

                    if (gamepad2.dpad_right && !movePlateRightToggle) {  // Move plate to the right, only works when array is emptied
                        movePlateRightToggle = true;
                        desiredPlatePoz -= 475;
                    } else movePlateRightToggle = false;

                    if (gamepad2.dpad_left && !movePlateLeftToggle) {  // Move plate to the left, only works when array is emptied
                        movePlateLeftToggle = true;
                        desiredPlatePoz += 475;
                    } else movePlateLeftToggle = false;


//                    if (gamepad2.x) {  // Forced outtake, used because regular outtake doesn't work after array is emptied
//                        if (!forcedOuttakeToggle) {
//                            forcedEmptyOnce = true;
//                            state = Intake_Sensor.FORCED_OUTTAKE;
//                            forcedOuttakeToggle = true;
//                        }
//                    } else forcedOuttakeToggle = false;


                    if (gamepad2.dpad_up) {  // Slightly move plate left in case of bad alignment after autonomous (hold)
                        desiredPlatePoz += 2;
                    }
                    if (gamepad2.dpad_down) {  // Slightly move plate right in case of bad alignment after autonomous (hold)
                        desiredPlatePoz -= 2;
                    }

                    if (gamepad2.y && !servoFail) {  // Forcefully resets lift position in case of failure
                        state = Intake_Sensor.INTAKE;
                        liftUp = false;
                        ShooterReady = false;
                        shootFound = false;
                        shot = false;
                        servoFail = true;
                    } else servoFail = false;

            if(gamepad1.y) {
                if (!intakeToggle) {
                    if (intakeOnce) intake.setDirection(DcMotorSimple.Direction.REVERSE);
                    else intake.setDirection(DcMotorSimple.Direction.FORWARD);

                    intakeOnce = !intakeOnce;
                    intakeToggle = true;
                }
            } else intakeToggle = false;


            plate.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            telemetry.addData("rateOfChange", rateOfChange);
            telemetry.addData("oldPos", oldVelocity);
            telemetry.addData("====================:", "");
            telemetry.addData("State: ", state);
            telemetry.addData("Motor Speed: ", outtakeRPM);
            telemetry.addData("toShoot: ", toShoot);
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
            telemetry.addData("currentPlatePoz:", plate.getCurrentPosition());
            telemetry.addData("axonencoder:", liftPoz);
            telemetry.update();
        }
    }
}