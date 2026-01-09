package org.firstinspires.ftc.teamcode.TeleOPs;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp (name = "TeleOP2")

public class  TeleOP2 extends LinearOpMode{

    DcMotor plate;
    DcMotor intake;
    DcMotorEx shooter;
    Servo lift;
    Servo camera;
    Servo servoShoot;
    DistanceSensor distanceSensor = null;
    NormalizedColorSensor test_color;

    Ball[] storage = {new Ball(), new Ball(), new Ball()};

    int plateTarget = 0;
    int ballsCollected = -1;
    boolean ballTaken = false;
    boolean full = false;
    int toShoot = 0;
    double avgColor;
    double hue = 0;
    public class Ball{
        public int pos = 0;
        public int color = 0;
    }
    boolean rightBumperToggle = false;
    boolean leftBumperToggle = true;

    boolean shooterReeady = false;
    boolean shot = false;
    boolean liftUp = false;
    int i = 2;


    public enum Intake_Sensor {
        EMPTY,
        ROTATE,
        OUTAKE,
        FORCED_OUTTAKE

    }

    SampleMecanumDrive drive;
    ElapsedTime intake_sensor_timer = new ElapsedTime();
    ElapsedTime motor_stop = new ElapsedTime();
    boolean shootFound = false;
    boolean rotateToggle = false;
    boolean servoFail = false;
    boolean dpadRightToggle = false;
    boolean dpadLeftToggle = false;
    boolean forcedEmpty = false;
    boolean forcedEmptyOnce = false;
    boolean ballShot = false;
    boolean gamepad2xToggle = false;
    int desiredPlatePoz = 0;
    boolean plateMoveForced = false;

    Intake_Sensor state = Intake_Sensor.EMPTY;

    AnalogInput encoderServo;
    AnalogInput encoderStopper;

    @Override
    public void runOpMode() throws InterruptedException {

        drive = new SampleMecanumDrive(hardwareMap);


        plate = hardwareMap.get(DcMotor.class, "plate");
        intake = hardwareMap.get(DcMotor.class, "intake");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");

        lift = hardwareMap.get(Servo.class, "lift");
        camera = hardwareMap.get(Servo.class, "servo_camera");
        servoShoot = hardwareMap.get(Servo.class, "servo_shoot");

        distanceSensor = hardwareMap.get(DistanceSensor.class, "distance");
        test_color = hardwareMap.get(NormalizedColorSensor.class, "color");

        encoderServo = hardwareMap.get(AnalogInput.class, "axon_encoder");
        encoderStopper = hardwareMap.get(AnalogInput.class, "stopper_encoder");

        double encoderPoz = 1-encoderServo.getVoltage()/3.3;
        double stopperPoz = 1-encoderStopper.getVoltage()/3.3;
        double servoSpeed = 1-encoderServo.getVoltage()/3.3 - encoderPoz;

        plate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        plate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        plate.setPower(0.7);
        plate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        plate.setTargetPosition(plateTarget);
        plate.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        double distance = distanceSensor.getDistance(DistanceUnit.MM);

        NormalizedRGBA colors = test_color.getNormalizedColors();

        waitForStart();

        intake_sensor_timer.reset();
        motor_stop.reset();

        while (opModeIsActive() && !isStopRequested()) {
            encoderPoz = 1-encoderServo.getVoltage()/3.3;
            stopperPoz = 1-encoderStopper.getVoltage()/3.3;
            servoSpeed = 1-encoderServo.getVoltage()/3.3 - encoderPoz;
            distance = distanceSensor.getDistance(DistanceUnit.MM);
            plate.setTargetPosition(plateTarget);

            drive.setWeightedDrivePower(new Pose2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x
            ));

            full = ballsCollected == 2;

            if(full) intake.setPower(0);

            switch(state){
                case EMPTY:
                    lift.setPosition(0.46);
                    if(!forcedEmptyOnce) intake.setPower(1);
                    servoShoot.setPosition(0.64);
                    if(ballsCollected == -1){
                        plateTarget = desiredPlatePoz;
                    }
                    if(distance < 45 && !full && plateTarget < plate.getCurrentPosition() + 5 && plateTarget > plate.getCurrentPosition() - 5 && !forcedEmptyOnce){

                        if(!ballTaken) {
                            ballsCollected += 1;
                            intake_sensor_timer.reset();
                            state = Intake_Sensor.ROTATE;
                            ballTaken = true;
                        } else ballTaken = false;

                        if(ballsCollected > 2) ballsCollected = 2;
                    }
                    if(motor_stop.seconds() >= 10){
                        shooter.setPower(0);
                    }
                    break;
                case ROTATE:
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

                        storage[ballsCollected].pos = plate.getCurrentPosition();
                        if(hue > 100) storage[ballsCollected].color = 2;
                        if(hue < 100) storage[ballsCollected].color = 1;

                    if(intake_sensor_timer. seconds() > 0.1){
                        plateTarget += 178;
                        state = Intake_Sensor.EMPTY;
                    }

                    if(motor_stop.seconds() >= 10){
                        shooter.setPower(0);
                    }

                    break;
                case OUTAKE:
                    intake.setPower(0);
                    boolean servoOpen = false;
                    shooter.setVelocity(1050);

                    if(shooter.getVelocity() > 1000){
                        shooterReeady = true;
                    }

                    if(!shootFound){
                        for(i = 2; i >= 0; i--){
                            if(storage[i].color == toShoot){
                                plateTarget = storage[i].pos + 178;
                                shootFound = true;
                                break;
                            }
                        }
                    }

                    if(shootFound && plate.getCurrentPosition() > plateTarget - 10 && plate.getCurrentPosition() < plateTarget + 10){
                        servoShoot.setPosition(0.47);
                        servoOpen = true;
                     }
                    if(servoOpen && shooterReeady && encoderPoz < 0.7 && !liftUp && stopperPoz < 0.375 && plate.getCurrentPosition() > plateTarget - 5 && plate.getCurrentPosition() < plateTarget + 5){
                        lift.setPosition(0.8);
                        liftUp = true;
                    }

                    if(encoderPoz > 0.7 && shooterReeady && !shot){
                        lift.setPosition(0.46);
                        shot = true;
                    }

                    if(shooterReeady && shooter.getVelocity() < 1000 && !ballShot && shot){
                        ballsCollected -= 1;
                        storage[i].pos = 0;
                        storage[i].color = 0;
                        ballShot = true;
                    }

                    if(encoderPoz < 0.5 && liftUp && shot) {
                        liftUp = false;
                        shooterReeady = false;
                        shootFound = false;
                        shot = false;
                        ballShot = false;
                        motor_stop.reset();
                        state = Intake_Sensor.EMPTY;
                    }

                    break;
                case FORCED_OUTTAKE:
                    intake.setPower(0);
                    servoOpen = false;
                    shooter.setVelocity(1050);

                    if(shooter.getVelocity() > 1000){
                        shooterReeady = true;
                    }

                    if(plate.getCurrentPosition() > plateTarget - 10 && plate.getCurrentPosition() < plateTarget + 10){
                        servoShoot.setPosition(0.47);
                        servoOpen = true;
                    }
                    if(servoOpen && shooterReeady && encoderPoz < 0.7 && !liftUp && stopperPoz < 0.375 && plate.getCurrentPosition() > plateTarget - 5 && plate.getCurrentPosition() < plateTarget + 5){
                        lift.setPosition(0.8);
                        liftUp = true;
                    }

                    if(encoderPoz > 0.7 && shooterReeady && !shot){
                        lift.setPosition(0.46);
                        shot = true;
                    }

                    if(shooterReeady && shooter.getVelocity() < 1000 && !ballShot && shot){
                        ballsCollected -= 1;
                        storage[i].pos = 0;
                        storage[i].color = 0;
                        ballShot = true;
                    }

                    if(encoderPoz < 0.5 && liftUp && shot) {
                        liftUp = false;
                        shooterReeady = false;
                        shootFound = false;
                        shot = false;
                        ballShot = false;
                        motor_stop.reset();
                        state = Intake_Sensor.EMPTY;
                    }
            }

            if(gamepad2.dpad_right && !dpadRightToggle){
                dpadRightToggle = true;
                desiredPlatePoz += 178;
                if(desiredPlatePoz > 500) desiredPlatePoz = 0;
            } else dpadRightToggle = false;

            if(gamepad2.dpad_left && !dpadLeftToggle){
                dpadLeftToggle = true;
                desiredPlatePoz -= 178;
                if(desiredPlatePoz < 0) desiredPlatePoz = 178*2;
            } else dpadLeftToggle = false;


            if(i == -1) i = 2;
            if(i == 3) i = 0;

            if(gamepad2.right_bumper && ballsCollected > -1){
                if(!rightBumperToggle)
                {
                    forcedEmptyOnce = true;
                    state = Intake_Sensor.OUTAKE;
                    toShoot = 2;
                    rightBumperToggle = true;
                }
            } else rightBumperToggle = false;

            if(gamepad2.left_bumper && ballsCollected > -1){
                if(!leftBumperToggle)
                {
                    forcedEmptyOnce = true;
                    state = Intake_Sensor.OUTAKE;
                    toShoot = 1;
                    leftBumperToggle = true;
                }
            } else leftBumperToggle = false;

            if(gamepad2.x){
                if(!gamepad2xToggle)
                {
                    forcedEmptyOnce = true;
                    state = Intake_Sensor.FORCED_OUTTAKE;
                    gamepad2xToggle = true;
                }
            } else gamepad2xToggle = false;

            if(gamepad2.a && !forcedEmpty){
                forcedEmptyOnce = false;
                state = Intake_Sensor.EMPTY;
                forcedEmpty = true;
                storage[0].pos = 0;
                storage[1].pos = 0;
                storage[2].pos = 0;
                storage[0].color = 0;
                storage[1].color = 0;
                storage[2].color = 0;
                ballsCollected = -1;
            } else forcedEmpty = false;

//            if(gamepad2.dpad_right && !dpadRightToggle){
//                i++;
//                if(i > 2) i = 0;
//                storage[i].color = 0;
//                plateTarget = storage[i].pos;
//                dpadRightToggle = true;
//            } else dpadRightToggle = false;
//
//            if(gamepad2.dpad_left && !dpadLeftToggle){
//                i--;
//                if(i < 0) i = 2;
//                storage[i].color = 0;
//                plateTarget = storage[i].pos;
//                dpadLeftToggle = true;
//            } else dpadLeftToggle = false;

            if(gamepad2.y && !servoFail){
                state = Intake_Sensor.EMPTY;
                liftUp = false;
                shooterReeady = false;
                shootFound = false;
                shot = false;
                servoFail = true;
            } else servoFail = false;

            if(ballsCollected < -1) ballsCollected = -1;

            plate.setMode(DcMotor.RunMode.RUN_TO_POSITION);

//            colors = test_color.getNormalizedColors();
//
//            double max = Math.max(colors.red, colors.blue);
//            double min = Math.min(colors.red, colors.blue);
//
//            max = Math.max(max, colors.green);
//            min = Math.min(min, colors.green);
//
//            double delta = max - min;
//
//            double hue = 0;
//
//            if(delta == 0) hue = 0;
//
//            if(max == colors.red) hue = 60 * (((colors.green - colors.blue) / delta) % 6);
//            if(max == colors.green) hue = 60 * ((colors.blue - colors.red) / delta);
//            if(max == colors.blue) hue = 60 * ((colors.red - colors.green) / delta);
//
//            if(hue < 0) hue += 360;

            telemetry.addData("====================:", "");
            telemetry.addData("State: ", state);
            telemetry.addData("Motor Speed: ", shooter.getVelocity());
            telemetry.addData("toShoot: ", toShoot);
            telemetry.addData("motorstop:", motor_stop.seconds());
            telemetry.addData("timer: ", intake_sensor_timer.seconds());
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
            telemetry.addData("avgcolor:", avgColor);
            telemetry.addData("BallsCollected:", ballsCollected);
            telemetry.addData("plateTarget:", plateTarget);
            telemetry.addData("axonencoder:", encoderPoz);
            telemetry.addData("shooterencoder:", stopperPoz);
            telemetry.addData("servoSpeed", servoSpeed);

            telemetry.update();
        }
    }
}