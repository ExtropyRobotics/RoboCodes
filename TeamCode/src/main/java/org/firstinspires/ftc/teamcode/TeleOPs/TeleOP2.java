package org.firstinspires.ftc.teamcode.TeleOPs;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.AnalogInput;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;

@TeleOp (name = "!TeleOP2")

public class  TeleOP2 extends LinearOpMode{
    public enum Intake_Sensor {
        INTAKE,
        ROTATE,
        OUTAKE,
        FORCED_OUTTAKE
    }

    public class Ball{
        public int pos = 0;
        public int color = 0;
    }
    Ball[] storage = {new Ball(), new Ball(), new Ball()};

    DcMotor plate;
    DcMotor intake;
    DcMotorEx shooter;
    Servo lift;
    Servo camera;
    Servo servoShoot;
    Servo frontPlateServo;
    Servo backPlateServo;
    DistanceSensor distanceSensor = null;
    NormalizedColorSensor test_color;

    int plateTarget = 0;
    int plateOffset = 0;
    int desiredPlatePoz = 0;
    int ballsCollected = -1;
    int i = 2;
    int toShoot = 0;

    double hue = 0;

    boolean purpleBallToggle = false;
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

    SampleMecanumDrive drive;
    Intake_Sensor state = Intake_Sensor.INTAKE;

    AnalogInput encoderLift;
    AnalogInput encoderStopper;

    @Override
    public void runOpMode() throws InterruptedException {

        drive = new SampleMecanumDrive(hardwareMap);

        // In correlation with Configuratie.txt (HOPEFULLY)

        plate = hardwareMap.get(DcMotor.class, "plate");
        intake = hardwareMap.get(DcMotor.class, "intake");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");

        lift = hardwareMap.get(Servo.class, "lift");
        camera = hardwareMap.get(Servo.class, "servo_camera");
        servoShoot = hardwareMap.get(Servo.class, "servo_shoot");

        frontPlateServo = hardwareMap.get(Servo.class, "rightPlateAdjuster");
        backPlateServo = hardwareMap.get(Servo.class, "leftPlateAdjuster");

        distanceSensor = hardwareMap.get(DistanceSensor.class, "distance");
        test_color = hardwareMap.get(NormalizedColorSensor.class, "color");

        encoderLift = hardwareMap.get(AnalogInput.class, "lift_encoder");
        encoderStopper = hardwareMap.get(AnalogInput.class, "stopper_encoder");

        plate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        plate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        plate.setPower(1);
        plate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        plate.setTargetPosition(plateTarget);
        plate.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        double distance;
        NormalizedRGBA colors;

        double liftPoz;
        double stopperPoz;

        while(opModeInInit() && !isStopRequested()){
            frontPlateServo.setPosition(0.5229);
            backPlateServo.setPosition(0.8729);
            plate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {

            frontPlateServo.setPosition(0.8007);
            backPlateServo.setPosition(0.1993);

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

            liftPoz = 1- encoderLift.getVoltage()/3.3;
            stopperPoz = 1-encoderStopper.getVoltage()/3.3;

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

            if(i == -1) i = 2;
            if(i == 3) i = 0;


            switch(state){
                case INTAKE:

                    lift.setPosition(0.46);
                    if(!forceOpenServo) {
                        servoShoot.setPosition(0.64);
                    }
                    if(!forcedEmptyOnce) intake.setPower(1);

                    if(ballsCollected == -1){
                        plateTarget = desiredPlatePoz + plateOffset;
                    }

                    if(distance < 45 && !full && plateTarget < plate.getCurrentPosition() + 5 && plateTarget > plate.getCurrentPosition() - 5 && !forcedEmptyOnce){
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

                    plateTarget += 178;
                    state = Intake_Sensor.INTAKE;

                    break;
                case OUTAKE:

                    intake.setPower(0);
                    shooter.setVelocity(1050);

                    servoOpen = false;

                    if(shooter.getVelocity() > 1000){
                        ShooterReady = true;
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
                    if(servoOpen && ShooterReady && liftPoz < 0.7 && !liftUp && stopperPoz < 0.375 && plate.getCurrentPosition() > plateTarget - 5 && plate.getCurrentPosition() < plateTarget + 5){
                        lift.setPosition(0.8);
                        liftUp = true;
                    }

                    if(liftPoz > 0.7 && ShooterReady && !shot){
                        lift.setPosition(0.46);
                        shot = true;
                    }

                    if(ShooterReady && shooter.getVelocity() < 1000 && !ballOut && shot){
                        ballsCollected -= 1;
                        storage[i].pos = 0;
                        storage[i].color = 0;
                        ballOut = true;
                    }

                    if(liftPoz < 0.5 && liftUp && shot) {
                        liftUp = false;
                        ShooterReady = false;
                        shootFound = false;
                        shot = false;
                        ballOut = false;
                        state = Intake_Sensor.INTAKE;
                    }

                    break;
                case FORCED_OUTTAKE:

                    intake.setPower(0);
                    shooter.setVelocity(1050);

                    servoOpen = false;

                    if(shooter.getVelocity() > 1000){
                        ShooterReady = true;
                    }

                    if(plate.getCurrentPosition() > plateTarget - 10 && plate.getCurrentPosition() < plateTarget + 10){
                        servoShoot.setPosition(0.47);
                        servoOpen = true;
                    }
                    if(servoOpen && ShooterReady && liftPoz < 0.7 && !liftUp && stopperPoz < 0.375 && plate.getCurrentPosition() > plateTarget - 5 && plate.getCurrentPosition() < plateTarget + 5){
                        lift.setPosition(0.8);
                        liftUp = true;
                    }

                    if(liftPoz > 0.7 && ShooterReady && !shot){
                        lift.setPosition(0.46);
                        shot = true;
                    }

                    if(ShooterReady && shooter.getVelocity() < 1000 && !ballOut && shot){
                        ballsCollected -= 1;
                        storage[i].pos = 0;
                        storage[i].color = 0;
                        ballOut = true;
                    }

                    if(liftPoz < 0.5 && liftUp && shot) {
                        liftUp = false;
                        ShooterReady = false;
                        shootFound = false;
                        shot = false;
                        ballOut = false;
                        state = Intake_Sensor.INTAKE;
                    }
            }



            if(gamepad2.a && !forcedEmpty){  // Empties the array, in case of wrong color detected, also makes it possible to move plate using dpads
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



            if(gamepad2.right_bumper && ballsCollected > -1){  // Rotate plate until ball is purple then shoot it
                if(!purpleBallToggle)
                {
                    forcedEmptyOnce = true;
                    state = Intake_Sensor.OUTAKE;
                    toShoot = 2;
                    purpleBallToggle = true;
                }
            } else purpleBallToggle = false;

            if(gamepad2.left_bumper && ballsCollected > -1){  // Rotate plate until ball is green then shoot it
                if(!greenBallToggle)
                {
                    forcedEmptyOnce = true;
                    state = Intake_Sensor.OUTAKE;
                    toShoot = 1;
                    greenBallToggle = true;
                }
            } else greenBallToggle = false;



            if(gamepad2.dpad_right && !movePlateRightToggle){  // Move plate to the right, only works when array is emptied
                movePlateRightToggle = true;
                desiredPlatePoz -= 178;
            } else movePlateRightToggle = false;

            if(gamepad2.dpad_left && !movePlateLeftToggle){  // Move plate to the left, only works when array is emptied
                movePlateLeftToggle = true;
                desiredPlatePoz += 178;
            } else movePlateLeftToggle = false;



            if(gamepad2.x){  // Forced outtake, used because regular outtake doesn't work after array is emptied
                if(!forcedOuttakeToggle)
                {
                    forcedEmptyOnce = true;
                    state = Intake_Sensor.FORCED_OUTTAKE;
                    forcedOuttakeToggle = true;
                }
            } else forcedOuttakeToggle = false;



            if(gamepad2.dpad_up){  // Slightly move plate left in case of bad alignment after autonomous (hold)
                desiredPlatePoz += 2;
            }
            if(gamepad2.dpad_down){  // Slightly move plate right in case of bad alignment after autonomous (hold)
                desiredPlatePoz -= 2;
            }



            if(gamepad2.b){  // Toggle the outtake motor manually, outside of state machine if needed (so you don't have to wait until 1050 RPM at basket)
                if(!manualOuttakeToggle){
                    if(manualOuttakeOnce) shooter.setPower(0);
                    else shooter.setVelocity(1050);

                    manualOuttakeOnce = !manualOuttakeOnce;
                    manualOuttakeToggle = true;
                }
            } else manualOuttakeToggle = false;



            if(gamepad2.y && !servoFail){  // Forcefully resets lift position in case of failure
                state = Intake_Sensor.INTAKE;
                liftUp = false;
                ShooterReady = false;
                shootFound = false;
                shot = false;
                servoFail = true;
            } else servoFail = false;



            if(gamepad1.right_bumper && !forceOpenServo){
                servoShoot.setPosition(0.47);
                forceOpenServo = true;
            } else forceOpenServo = false;

            plate.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            telemetry.addData("====================:", "");
            telemetry.addData("State: ", state);
            telemetry.addData("Motor Speed: ", shooter.getVelocity());
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
            telemetry.addData("currentPlatePoz:" , plate.getCurrentPosition());
            telemetry.addData("axonencoder:", liftPoz);
            telemetry.addData("shooterencoder:", stopperPoz);
//
            telemetry.update();
        }
    }
}