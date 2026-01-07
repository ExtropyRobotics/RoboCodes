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

    double timer = 1;
    int plateTarget = 0;
    int ballsCollected = -1;
    boolean ballTaken = false;
    boolean full = false;
    boolean slowShoot = false;
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


    public enum Intake_Sensor {
        EMPTY,
        ROTATE,
        OUTAKE

    }

    SampleMecanumDrive drive;
    ElapsedTime intake_sensor_timer = new ElapsedTime();
    ElapsedTime motor_speed = new ElapsedTime();
    ElapsedTime motor_stop = new ElapsedTime();
    ElapsedTime servo_wait = new ElapsedTime();
    boolean resetOnce = false;
    boolean shootFound = false;
    boolean shootHold = false;
    boolean servoUp = false;
    boolean servoDown = false;
    boolean servoMove = false;
    boolean motorSpeedResetToggle = false;
    boolean outtakeReady = false;
    boolean outtakeSwitch = false;
    boolean rotateToggle = false;
    boolean intakeToggle = false;
    boolean intakeOnce = false;
    Intake_Sensor state = Intake_Sensor.EMPTY;

    AnalogInput encoderServo;

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

        double encoderPoz = 1-encoderServo.getVoltage()/3.3;

        plate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        plate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        plate.setPower(1);
        plate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        plate.setTargetPosition(plateTarget);
        plate.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        double distance = distanceSensor.getDistance(DistanceUnit.CM);

        NormalizedRGBA colors = test_color.getNormalizedColors();

        waitForStart();

        intake_sensor_timer.reset();
        motor_speed.reset();
        motor_stop.reset();
        servo_wait.reset();

        while (opModeIsActive() && !isStopRequested()) {
            encoderPoz = 1-encoderServo.getVoltage()/3.3;
            distance = distanceSensor.getDistance(DistanceUnit.CM);
            plate.setTargetPosition(plateTarget);

            drive.setWeightedDrivePower(new Pose2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x
            ));

            switch(state){

                case EMPTY:
                    servoShoot.setPosition(0.5);
                    if(ballsCollected == -1 && !rotateToggle){
                        plateTarget = 0;
                    }
                    if(distance < 15 && !full){

                        if(!ballTaken) {
                            ballsCollected += 1;
                            intake_sensor_timer.reset();
                            state = Intake_Sensor.ROTATE;
                            ballTaken = true;
                        } else ballTaken = false;

                        if(ballsCollected > 2) ballsCollected = 2;
                    }
                    if(shootHold && motor_stop.seconds() >= 13){
                        shooter.setPower(0);
                        shootHold = false;
                    }
                    break;
                case ROTATE:
                    if(intake_sensor_timer.seconds() <= 0.5){
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
                        if(hue > 250) storage[ballsCollected].color = 2;
                        if(hue < 250) storage[ballsCollected].color = 1;


                    }
                    if(intake_sensor_timer.seconds() > 0.5){
                        state = Intake_Sensor.EMPTY;
                        plateTarget += 179;
                    }
                    if(shootHold && motor_stop.seconds() >= 13){
                        shooter.setPower(0);
                        shootHold = false;
                    }

                    break;
                case OUTAKE:
                    boolean servoOpen = false;
                    shooter.setVelocity(1050);

                    if(shooter.getVelocity() > 1000){
                        shooterReeady = true;
                    }

                    if(!shootFound){
                        for(int i = 2; i >= 0 && !shootFound; i--){
                            if(storage[i].color == toShoot){
                                plateTarget = storage[i].pos + 179 + 20;
                                storage[i].pos = 0;
                                storage[i].color = 0;
                                shootFound = true;
                            }
                        }
                    }

                    if(shootFound && plate.getCurrentPosition() > plateTarget - 10 && plate.getCurrentPosition() < plateTarget + 10){
                        servoShoot.setPosition(0.37);
                        servoOpen = true;
                    }
                    if(servoOpen && shooterReeady && encoderPoz < 0.7 && !liftUp){
                        lift.setPosition(0.8);
                        liftUp = true;
                    }
                    if(encoderPoz > 0.7 && shooterReeady && !shot){
                        lift.setPosition(0.46);
                        shot = true;
                    }
                    if(encoderPoz < 0.5 && liftUp && shot){
                        state = Intake_Sensor.EMPTY;
                        liftUp = false;
                        shooterReeady = false;
                        shootFound = false;
                        shot = false;
                    }




//                    boolean plateReady = false;
//                    if(!motorSpeedResetToggle){
//                    motorSpeedResetToggle = true;
//                    motor_speed.reset();
//                    }
//                    shooter.setVelocity(1000);
//                    intake.setPower(0);
//                    if(intake_sensor_timer.seconds() < timer && !shootFound){
//                        for(int i = 2; i >= 0 && !shootFound; i--){
//                            if(storage[i].color == toShoot){
//                                plateTarget = storage[i].pos + 179 + 20;
//                                storage[i].pos = 0;
//                                storage[i].color = 0;
//                                shootFound = true;
//                            }
//                        }
//                    }
//
//                    if(plate.getCurrentPosition() > plateTarget - 10 && plate.getCurrentPosition() < plateTarget + 10){
//                        servoShoot.setPosition(0.37);
//                        plateReady = true;
//                    }
//
//                    if(!resetOnce && shooter.getVelocity() >= 1000){
//                        resetOnce = true;
//                        slowShoot = true;
//                        servoMove = true;
//                    }
//                    if(servoMove && shooter.getVelocity() >= 1000 && motor_speed.seconds() >= 2 && !servoUp && plateReady){
//                        lift.setPosition(0.8);
//                        servoUp = true;
//                        motor_speed.reset();
//                        servoMove = false;
//                        resetOnce = true;
//                        outtakeSwitch = true;
//                    }
//                    if(shooter.getVelocity() < 1000 && slowShoot){
//                        ballsCollected -= 1;
//                        lift.setPosition(0.46);
//                        slowShoot = false;
//                    }
//                    if(motor_speed.seconds() >= 0.5 && resetOnce && !servoDown){
//                        shootHold = true;
//                        outtakeReady = true;
//                        servoDown = true;
//                    }
//                    if(motor_speed.seconds() >= 1.5 && resetOnce && outtakeReady && outtakeSwitch && encoderPoz < 0.5){
//                        shootFound = false;
//                        resetOnce = false;
//                        outtakeReady = false;
//                        motorSpeedResetToggle = false;
//                        outtakeSwitch = false;
//                        slowShoot = false;
//                        servoDown = false;
//                        servoUp = false;
//                        intake_sensor = Intake_Sensor.EMPTY;
//                    }
                    break;
            }

            if(ballsCollected == 2) intake.setPower(0);

            if(gamepad2.right_bumper){
                if(!rightBumperToggle)
                {
                    intake_sensor_timer.reset();
                    state = Intake_Sensor.OUTAKE;
                    toShoot = 2;
                    rightBumperToggle = true;
                }
            } else rightBumperToggle = false;

            if(gamepad2.left_bumper){
                if(!leftBumperToggle)
                {
                    intake_sensor_timer.reset();
                    state = Intake_Sensor.OUTAKE;
                    toShoot = 1;
                    leftBumperToggle = true;
                }
            } else leftBumperToggle = false;

            if(ballsCollected == -2) ballsCollected = -1;

            if(gamepad2.dpad_up && !rotateToggle){
                plateTarget = 179;
                rotateToggle = true;
            } else rotateToggle = false;

            if(gamepad2.y){
                if(!intakeToggle){
                    if(intakeOnce) intake.setPower(0);
                    else intake.setPower(1);

                    intakeOnce = !intakeOnce;
                    intakeToggle = true;
                }
            } else intakeToggle = false;

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
            telemetry.addData("motorspeed", motor_speed.seconds());
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
            telemetry.addData("Distance: ", distanceSensor.getDistance(DistanceUnit.CM));
            telemetry.addData("hue: ", hue);
            telemetry.addData("avgcolor:", avgColor);
            telemetry.addData("BallsCollected:", ballsCollected);
            telemetry.addData("plateTarget:", plateTarget);
            telemetry.addData("axonencoder:", encoderPoz);

            telemetry.update();
        }
    }
}