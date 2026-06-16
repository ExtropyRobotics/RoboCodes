package org.firstinspires.ftc.teamcode.Autonomii.tests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.List;
@Disabled
@Autonomous (name = "!testAuto")

public class autoTest extends LinearOpMode {
    volatile int a = 0;
    DcMotor plate;
    DcMotor intake;
    DcMotorEx shooter;
    Servo lift;
    Servo camera;
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
    class Ball{
        int pos = 0;
        int color = 0;
    }
    boolean rightBumperToggle = false;
    boolean leftBumperToggle = true;

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
    Intake_Sensor intake_sensor = Intake_Sensor.EMPTY;
    double distance;

    Limelight3A limelight;

    LLResult llResult;
    int aprilTagID = 0;
    NormalizedRGBA colors;

    public void movePlate(int plateTarget){
        plate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        plate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        plate.setPower(0.7);
        plate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        plate.setTargetPosition(plateTarget);
        plate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    class stateMachine implements Runnable{

        @Override
        public void run() {
            while (opModeIsActive() && !isStopRequested()) {
                distance = distanceSensor.getDistance(DistanceUnit.CM);
                plate.setTargetPosition(plateTarget);

                switch(intake_sensor){

                    case EMPTY:
                        if(ballsCollected == -1 && !rotateToggle){
                            plateTarget = 0;
                        }
                        lift.setPosition(0.46);
                        if(distance < 15 && !full){

                            if(!ballTaken) {
                                ballsCollected += 1;
                                intake_sensor_timer.reset();
                                intake_sensor = Intake_Sensor.ROTATE;
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
                            intake_sensor = Intake_Sensor.EMPTY;
                            plateTarget += 179;
                        }
                        if(shootHold && motor_stop.seconds() >= 13){
                            shooter.setPower(0);
                            shootHold = false;
                        }

                        break;
                    case OUTAKE:
                        if(!motorSpeedResetToggle){
                            motorSpeedResetToggle = true;
                            motor_speed.reset();
                        }
                        shooter.setVelocity(1000);
                        intake.setPower(0);
                        if(intake_sensor_timer.seconds() < timer && !shootFound){
                            for(int i = 0; i < 3 && !shootFound; i++){
                                if(storage[i].color == toShoot){
                                    plateTarget = storage[i].pos + 179 + 20;
                                    storage[i].pos = 0;
                                    storage[i].color = 0;
                                    shootFound = true;
                                }
                            }
                        }

                        if(!resetOnce && shooter.getVelocity() >= 1000){
                            resetOnce = true;
                            slowShoot = true;
                            servoMove = true;
                        }
                        if(servoMove && shooter.getVelocity() >= 1000 && motor_speed.seconds() >= 2 && !servoUp){
                            lift.setPosition(0.8);
                            servoUp = true;
                            motor_speed.reset();
                            servoMove = false;
                            resetOnce = true;
                            outtakeSwitch = true;
                        }
                        if(shooter.getVelocity() < 1000 && slowShoot){
                            ballsCollected -= 1;
                            slowShoot = false;
                        }
                        if(motor_speed.seconds() >= 0.5 && resetOnce && !servoDown){
                            shootHold = true;
                            outtakeReady = true;
                            servoDown = true;
                        }
                        if(motor_speed.seconds() >= 1.5 && resetOnce && outtakeReady && outtakeSwitch){
                            intake_sensor = Intake_Sensor.EMPTY;
                            shootFound = false;
                            resetOnce = false;
                            outtakeReady = false;
                            motorSpeedResetToggle = false;
                            outtakeSwitch = false;
                            slowShoot = false;
                            servoDown = false;
                            servoUp = false;
                        }
                        break;
                }

                llResult = limelight.getLatestResult();
                if(llResult != null && llResult.isValid()) {
                    List<LLResultTypes.FiducialResult> fiducialResults = llResult.getFiducialResults();
                    for (LLResultTypes.FiducialResult fr : fiducialResults) {
                        telemetry.addData("ID", fr.getFiducialId());
                        aprilTagID = fr.getFiducialId();
                    }
                }

            telemetry.addData("====================:", "");
            telemetry.addData("State: ", intake_sensor);
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
            telemetry.addData("BallsCollected:", ballsCollected);
            telemetry.addData("plateTarget:", plateTarget);

            telemetry.update();
            if(ballsCollected == -2) ballsCollected = -1;
        }
    }
}

    stateMachine obj = new stateMachine();

    @Override
    public void runOpMode() throws InterruptedException {
        Thread thread = new Thread(obj);

        ballsCollected = 2;
        storage[0].color = 1;
        storage[1].color = 2;
        storage[2].color = 2;
        storage[0].pos = 0;
        storage[1].pos = 179;
        storage[2]. pos = 179*2;

        drive = new SampleMecanumDrive(hardwareMap);

        plate = hardwareMap.get(DcMotor.class, "plate");
        intake = hardwareMap.get(DcMotor.class, "intake");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        lift = hardwareMap.get(Servo.class, "lift");
        camera = hardwareMap.get(Servo.class, "servo_camera");

        distanceSensor = hardwareMap.get(DistanceSensor.class, "distance");
        test_color = hardwareMap.get(NormalizedColorSensor.class, "color");

        limelight = hardwareMap.get(Limelight3A .class, "limelight");
        limelight.pipelineSwitch(0);

        plate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        plate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        plate.setPower(0.1);
        plate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        plate.setTargetPosition(plateTarget);
        plate.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        intake_sensor_timer.reset();
        motor_speed.reset();
        motor_stop.reset();
        servo_wait.reset();

        Pose2d startingPose = new Pose2d(-51, -44, Math.toRadians(-125));
        TrajectorySequence DecodeAuto = drive.trajectorySequenceBuilder(startingPose)
                .setTangent(Math.toRadians(45))
                .setVelConstraint(new TranslationalVelocityConstraint(5))
                .splineToConstantHeading(new Vector2d(-31,-21),Math.toRadians(45))

                .waitSeconds(10)
                .build();

        drive.setPoseEstimate(startingPose);

        limelight.start();
        thread.start();
        waitForStart();
        sleep(500);
        drive.followTrajectorySequence(DecodeAuto);
        sleep(30000);
    }
}