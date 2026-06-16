package org.firstinspires.ftc.teamcode.Autonomii.regio;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.List;

@Disabled
@Autonomous (name = "!closeBlue")

public class ACTUALregioLeftBlue extends LinearOpMode {DcMotorEx plateEncoder;
    CRServo plateServoLeft;
    CRServo plateServoRight;
    DcMotorEx intake;
    DcMotorEx outtake;
    DcMotorEx outtake2;
    NormalizedColorSensor colorSensor;
    DistanceSensor distanceSensor;
    DcMotorEx turret = null;
    Limelight3A limelight;
    Pose2d startingPose = new Pose2d(-51, 44, Math.toRadians(-125));

    int sumPlate = 0;
    int encoderPos = 0;
    double plateError = 0;
    ElapsedTime timer = new ElapsedTime();
    int plateTargetPosition = 0;
    double platePow = 1;
    double maxPlatePower = 0.6;
    double plateP = 0.0002;
    double plateI = 0.00000;
    double plateD = 0.0008;

    class autoThread implements Runnable {
        @Override
        public void run() {

            while (opModeIsActive() && !isStopRequested()) {
//                double plateDt = timer.seconds();
//
//                encoderPos = -plateEncoder.getCurrentPosition();
//                plateError = plateTargetPosition - encoderPos;
//                sumPlate += (int) (plateError * plateDt);
//                double deriv = (plateTargetPosition - encoderPos - plateError);
//
//                platePow = plateP*plateError + plateI*sumPlate + deriv* plateD;
//
//                if(platePow < -maxPlatePower) platePow = -maxPlatePower;
//                if(platePow > maxPlatePower) platePow = maxPlatePower;
//
//                plateServoRight.setPower(platePow);
//                plateServoLeft.setPower(platePow);
            }
        }
    }

    autoThread obj = new autoThread();

    @Override
    public void runOpMode(){
        Thread thread = new Thread(obj);
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(),telemetry);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        plateServoLeft = hardwareMap.get(CRServo.class, "plateServoLeft");
        plateServoRight = hardwareMap.get(CRServo.class, "plateServoRight");

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        outtake = hardwareMap.get(DcMotorEx.class, "outtake");
        turret = hardwareMap.get(DcMotorEx.class, "turret");

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");

        plateEncoder = hardwareMap.get(DcMotorEx.class, "intake");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        outtake2 = hardwareMap.get(DcMotorEx.class, "outtake2");

        outtake.setDirection(DcMotorSimple.Direction.REVERSE);
        outtake2.setDirection(DcMotorSimple.Direction.REVERSE);

        TrajectorySequence DecodeAuto= drive.trajectorySequenceBuilder(startingPose)
                .UNSTABLE_addTemporalMarkerOffset(0, ()->{
                    outtake.setVelocity(1200);
                    outtake2.setVelocity(1200);
                    intake.setPower(1);
                })
                .back(47)
                .turn(Math.toRadians(10))
                .UNSTABLE_addTemporalMarkerOffset(2, ()->{
                    plateServoLeft.setPower(-0.6);
                    plateServoRight.setPower(-0.6);
                })
                .UNSTABLE_addTemporalMarkerOffset(3, ()->{
                    plateServoLeft.setPower(0);
                    plateServoRight.setPower(0);
                })
                .UNSTABLE_addTemporalMarkerOffset(2.8+1, ()->{
                    plateServoLeft.setPower(-0.6);
                    plateServoRight.setPower(-0.6);
                })
                .UNSTABLE_addTemporalMarkerOffset(3.1+1, ()->{
                    plateServoLeft.setPower(0);
                    plateServoRight.setPower(0);
                })
                .forward(5)
                .UNSTABLE_addTemporalMarkerOffset(7, ()->{
                    plateServoLeft.setPower(-0.6);
                    plateServoRight.setPower(-0.6);
                })
                .waitSeconds(200)
                .build();

        drive.setPoseEstimate(startingPose);

        limelight.start();
        timer.reset();
        waitForStart();
        thread.start();
        drive.followTrajectorySequence(DecodeAuto);
    }

    @Disabled
    @Autonomous (name = "!AutoLeftRED")

    public static class AutoLeftRed extends LinearOpMode {
        public void movePlate(int plateTarget, int power){
            plate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            plate.setPower(power);
            plate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            plate.setTargetPosition(plateTarget);
            plate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        DcMotorEx turret = null;
        DcMotor plate = null;
        int rotatePlate = 475;
        DcMotor intake = null;
        DcMotorEx shooter = null;
        Servo lift = null;
        double liftUp = 1;
        double liftDown = 0.55;
        int motorRPM = 1200;
        Limelight3A limelight;
        LLResult llResult;
        int aprilTagID = 0;
        //    boolean firstAprilTag = false;
        NormalizedColorSensor colorSensor;
        DistanceSensor distanceSensor;
        Pose2d startingPose = new Pose2d(-51, 44, Math.toRadians(155));


        class autoThread implements Runnable {
            @Override
            public void run() {

                turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                turret.setPower(1);
                turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                turret.setTargetPosition(0);
                turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                while(opModeInInit() && !isStopRequested()){
                    plate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }

                while (opModeIsActive() && !isStopRequested()) {
    //                llResult = limelight.getLatestResult();

                    turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    //                if (llResult != null && llResult.isValid()) {
    //                    limelight.pipelineSwitch(0);
    //                    List<LLResultTypes.FiducialResult> fiducialResults = llResult.getFiducialResults();
    //                    for (LLResultTypes.FiducialResult fr : fiducialResults) {
    //                        telemetry.addData("ID", fr.getFiducialId());
    //                        aprilTagID = fr.getFiducialId();
    //                        telemetry.update();
    //                    }
    //                }
                }
            }
        }

        autoThread obj = new autoThread();

        @Override
        public void runOpMode(){
            Thread thread = new Thread(obj);
            telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(),telemetry);

            SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

            plate = hardwareMap.get(DcMotor.class, "plate");
            intake = hardwareMap.get(DcMotor.class, "intake");
            shooter = hardwareMap.get(DcMotorEx.class, "outtake");

            turret = hardwareMap.get(DcMotorEx.class, "aim");

            lift = hardwareMap.get(Servo.class, "lift");

            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            distanceSensor = hardwareMap.get(DistanceSensor.class, "distance");
            colorSensor = hardwareMap.get(NormalizedColorSensor.class, "color");

            TrajectorySequence DecodeAuto= drive.trajectorySequenceBuilder(startingPose)
                    .UNSTABLE_addTemporalMarkerOffset(0, ()->{
                        shooter.setVelocity(motorRPM);
                        lift.setPosition(0.55);
                    })
                    .setTangent(Math.toRadians(-45))
                    .splineToConstantHeading(new Vector2d(-18,8),Math.toRadians(-45))
                    .UNSTABLE_addTemporalMarkerOffset(1, ()->{
                        movePlate(rotatePlate + 237, 1);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(3, ()->{
                        lift.setPosition(liftUp);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(5, ()->{
                        lift.setPosition(liftDown);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(6, ()->{
                        movePlate(plate.getCurrentPosition() + rotatePlate, 1);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(7, ()->{
                        lift.setPosition(liftUp);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(8, ()->{
                        lift.setPosition(liftDown);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(9, ()->{
                        movePlate(plate.getCurrentPosition() + rotatePlate, 1);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(11, ()->{
                        lift.setPosition(liftUp);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(12, ()->{
                        lift.setPosition(liftDown);
                        shooter.setVelocity(0);
                        intake.setPower(1);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(14, ()->{
                        movePlate(0, 1);
                    })
                    .waitSeconds(200)
    //                .waitSeconds(5)
    //                .setTangent(Math.toRadians(0))
    //                .splineToSplineHeading(new Pose2d(-16, -8, Math.toRadians(-90)), Math.toRadians(0))
    //                .splineToConstantHeading(new Vector2d(-9.7, -20), Math.toRadians(-90))
    //                .setVelConstraint(new TranslationalVelocityConstraint(17))
    //                .UNSTABLE_addTemporalMarkerOffset(0.35, ()->{
    //                    movePlate(rotatePlate, 1);
    //                    shooter.setVelocity(motorRPM);
    //                })
    //                .UNSTABLE_addTemporalMarkerOffset(0.8, ()->{
    //                    movePlate(rotatePlate * 2, 1);
    //                })
    //                .splineToConstantHeading(new Vector2d(-9.7, -49), Math.toRadians(-90))
    //                .UNSTABLE_addTemporalMarkerOffset(0.2, ()->{
    //                  movePlate(0, 1);
    //                })
    //                .resetVelConstraint()
    //                .UNSTABLE_addTemporalMarkerOffset(0.2, ()->{
    ////                    if(aprilTagID == 22) movePlate(rotatePlate * 2 + 237, 1);
    ////                    if(aprilTagID == 23) movePlate(rotatePlate + 237, 1);
    //                    movePlate(rotatePlate + 237, 1);
    //                })
    //                .setTangent(Math.toRadians(90))
    //                .splineToSplineHeading(new Pose2d(-18, -8, Math.toRadians(-125)), Math.toRadians(180))
    //                .UNSTABLE_addTemporalMarkerOffset(0.3, ()->{
    //                })
    //                .UNSTABLE_addTemporalMarkerOffset(0.6, ()->{
    //                    intake.setPower(0);
    //                    lift.setPosition(liftUp);
    //                })
    //                .UNSTABLE_addTemporalMarkerOffset(1.2, ()->{
    //                    lift.setPosition(liftDown);
    //                })
    //                .UNSTABLE_addTemporalMarkerOffset(1.4, ()->{
    //                    movePlate(plate.getCurrentPosition() + rotatePlate, 1);
    //                })
    //                .UNSTABLE_addTemporalMarkerOffset(2.5, ()->{
    //                    lift.setPosition(liftUp);
    //                })
    //                .UNSTABLE_addTemporalMarkerOffset(2.9, ()->{
    //                    lift.setPosition(liftDown);
    //                })
    //                .UNSTABLE_addTemporalMarkerOffset(3.1, ()->{
    //                    movePlate(plate.getCurrentPosition() + rotatePlate, 1);
    //                })
    //                .UNSTABLE_addTemporalMarkerOffset(4.2, ()->{
    //                    lift.setPosition(liftUp);
    //                })
    //                .UNSTABLE_addTemporalMarkerOffset(4.6, ()->{
    //                    lift.setPosition(liftDown);
    //                    shooter.setVelocity(0);
    //                    intake.setPower(1);
    //                })
    //                .UNSTABLE_addTemporalMarkerOffset(5.1, ()->{
    //                    movePlate(0, 1);
    //                })
    //                .waitSeconds(4.6)
    //                .setTangent(0)
    //                .splineToSplineHeading(new Pose2d(5, -14, Math.toRadians(-110)), Math.toRadians(0))
    //                .splineToConstantHeading(new Vector2d(12, -14), Math.toRadians(0))
    //                .splineToConstantHeading(new Vector2d(17.2, -22), Math.toRadians(-90))
    //                .setVelConstraint(new TranslationalVelocityConstraint(17))
    //                .UNSTABLE_addTemporalMarkerOffset(0.3, ()->{
    //                    movePlate(rotatePlate, 1);
    //                    shooter.setVelocity(motorRPM);
    //                })
    //                .UNSTABLE_addTemporalMarkerOffset(0.7, ()->{
    //                    movePlate(rotatePlate * 2, 1);
    //                })
    //                .splineToConstantHeading(new Vector2d(17.2, -54), Math.toRadians(-90))
    //                .resetVelConstraint()
    //                .UNSTABLE_addTemporalMarkerOffset(0.4, ()->{
    ////                    if(aprilTagID == 22) movePlate((rotatePlate* 4 + 237), 1);
    ////                    if(aprilTagID == 23) movePlate(rotatePlate * 3 + 237, 1);
    ////                    if(aprilTagID == 21) movePlate(rotatePlate + 237, 1);
    //                    movePlate(rotatePlate + 237, 1);
    //                })
    //                .setTangent(Math.toRadians(90))
    //                .splineToSplineHeading(new Pose2d(-20, -8, Math.toRadians(-125)), Math.toRadians(180))
    //                .UNSTABLE_addTemporalMarkerOffset(0.7, ()->{
    //                })
    //                .UNSTABLE_addTemporalMarkerOffset(1.1, ()->{
    //                    intake.setPower(0);
    //                    lift.setPosition(liftUp);
    //                    shooter.setVelocity(motorRPM);
    //                })
    //                .UNSTABLE_addTemporalMarkerOffset(1.6, ()->{
    //                    lift.setPosition(liftDown);
    //                })
    //                .UNSTABLE_addTemporalMarkerOffset(1.8, ()->{
    //                    movePlate(plate.getCurrentPosition() + rotatePlate, 1);
    //                    shooter.setVelocity(motorRPM);
    //                })
    //                .UNSTABLE_addTemporalMarkerOffset(2.2, ()->{
    //                    lift.setPosition(liftUp);
    //                })
    //                .UNSTABLE_addTemporalMarkerOffset(2.7, ()->{
    //                    lift.setPosition(liftDown);
    //                    shooter.setVelocity(motorRPM-70);
    //                })
    //                .UNSTABLE_addTemporalMarkerOffset(3, ()->{
    //                    movePlate(plate.getCurrentPosition() + rotatePlate, 1);
    //                })
    //                .UNSTABLE_addTemporalMarkerOffset(3.5, ()->{
    //                    lift.setPosition(liftUp);
    //                })
    //                .UNSTABLE_addTemporalMarkerOffset(4, ()->{
    //                    lift.setPosition(liftDown);
    //                    shooter.setVelocity(0);
    //                })
    //                .waitSeconds(200)
                    .build();

            drive.setPoseEstimate(startingPose);

            limelight.start();

            thread.start();
            waitForStart();
            drive.followTrajectorySequence(DecodeAuto);
        }
    }

    @Disabled
    @Autonomous (name = "!AutoRightBlue")

    public static class AutoRightBlue extends LinearOpMode {
        public void movePlateID(int plateTarget, double power){
            plate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            plate.setPower(power);
            plate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            plate.setTargetPosition(plateTarget);
            plate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        public void movePlate(int plateTarget){
            plate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            plate.setPower(1);
            plate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            plate.setTargetPosition(plateTarget);
            plate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        public void movePlateGather(int plateTarget){
            plate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            plate.setPower(1);
            plate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            plate.setTargetPosition(plateTarget);
            plate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        DcMotor plate = null;
        int rotatePlate = 176;
        DcMotor intake = null;
        DcMotorEx shooter = null;
        Servo lift = null;
        double liftUp = 0.8;
        double liftDown = 0.46;
        Servo camera = null;
        Servo servoShoot = null;
        int motorRPM = 1055;
        double servoShootOpen = 0.47;
        double servoShootClose = 0.64;
        Limelight3A limelight;
        LLResult llResult;
        int aprilTagID = 0;
        Servo frontPlateServo;
        Servo backPlateServo;
        boolean firstAprilTag = false;
        NormalizedColorSensor colorSensor;
        DistanceSensor distanceSensor;
        Pose2d startingPose = new Pose2d(58, -10, Math.toRadians(180));

        class autoThread implements Runnable {
            @Override
            public void run() {

                while(opModeInInit() && !isStopRequested()){
                    frontPlateServo.setPosition(0.5229);
                    backPlateServo.setPosition(0.8729);
                    plate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }

                while (opModeIsActive() && !isStopRequested()) {
                    llResult = limelight.getLatestResult();

                    if (llResult != null && llResult.isValid()) {
                        limelight.pipelineSwitch(0);
                        List<LLResultTypes.FiducialResult> fiducialResults = llResult.getFiducialResults();
                        for (LLResultTypes.FiducialResult fr : fiducialResults) {
                            telemetry.addData("ID", fr.getFiducialId());
                            aprilTagID = fr.getFiducialId();
                            telemetry.update();
                        }
                        camera.setPosition(0.45);
                    }
                }
            }
        }

        autoThread obj = new autoThread();

        @Override
        public void runOpMode(){
            Thread thread = new Thread(obj);
            telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(),telemetry);

            SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

            plate = hardwareMap.get(DcMotor.class, "plate");
            intake = hardwareMap.get(DcMotor.class, "intake");
            shooter = hardwareMap.get(DcMotorEx.class, "shooter");

            lift = hardwareMap.get(Servo.class, "lift");
            camera = hardwareMap.get(Servo.class, "servo_camera");
            servoShoot = hardwareMap.get(Servo.class, "servo_shoot");

            frontPlateServo = hardwareMap.get(Servo.class, "rightPlateAdjuster");
            backPlateServo = hardwareMap.get(Servo.class, "leftPlateAdjuster");

            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            distanceSensor = hardwareMap.get(DistanceSensor.class, "distance");
            colorSensor = hardwareMap.get(NormalizedColorSensor.class, "color");

            TrajectorySequence DecodeAuto= drive.trajectorySequenceBuilder(startingPose)
                    .setTangent(Math.toRadians(-115))
                    .UNSTABLE_addTemporalMarkerOffset(0, ()->{
                        shooter.setVelocity(1350);
                        camera.setPosition(0.76);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(0.3, ()->{
                        frontPlateServo.setPosition(0.8007);
                        backPlateServo.setPosition(0.1993);
                    })
                    .setTangent(Math.toRadians(180))
                    .splineToConstantHeading(new Vector2d(54, -10), Math.toRadians(180))
                    .splineToSplineHeading(new Pose2d(50, -15, Math.toRadians(-155)), Math.toRadians(-90))
                    .UNSTABLE_addTemporalMarkerOffset(1.5, ()->{
                        lift.setPosition(0.8);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(2.5, ()->{
                        lift.setPosition(0.46);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(2.7, ()->{
                        movePlate(plate.getCurrentPosition() + rotatePlate);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(3.7, ()->{
                        lift.setPosition(0.8);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(4.2, ()->{
                        lift.setPosition(0.46);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(4.4, ()->{
                        movePlate(plate.getCurrentPosition() + rotatePlate);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(5.4, ()->{
                        lift.setPosition(0.8);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(5.9, ()->{
                        lift.setPosition(0.46);
                        shooter.setVelocity(0);
                        intake.setPower(0);
                    })
    //                .waitSeconds(5.6)
    //                .setTangent(180)
    //                .splineToSplineHeading(new Pose2d(36, -25, Math.toRadians(-90)), Math.toRadians(-90))
    //                .setVelConstraint(new TranslationalVelocityConstraint(18))
    //                .UNSTABLE_addTemporalMarkerOffset(0.6, ()->{
    //                    movePlateID(rotatePlate, 1);
    //                })
    //                .UNSTABLE_addTemporalMarkerOffset(1.1, ()->{
    //                    movePlateID(rotatePlate, 1);
    //                })
    //                .UNSTABLE_addTemporalMarkerOffset(1.7, ()->{
    //                    shooter.setVelocity(1000);
    //                })
    //                .UNSTABLE_addTemporalMarkerOffset(2.2, ()->{
    //                    if(aprilTagID == 21) movePlateID(rotatePlate*2, 1);
    //                    if(aprilTagID == 22) movePlateID(rotatePlate, 1);
    //                })
    //                .UNSTABLE_addTemporalMarkerOffset(2.8, ()->{
    //                    intake.setPower(0);
    //                    shooter.setVelocity(1350);
    //                })
    //                .splineToConstantHeading(new Vector2d(36, -60), Math.toRadians(-90))
    //                .resetVelConstraint()
    //                .setTangent(Math.toRadians(90))
    //                .splineToSplineHeading(new Pose2d(50, -15, Math.toRadians(-155)), Math.toRadians(0))
    //                .UNSTABLE_addTemporalMarkerOffset(1, ()->{
    //                    lift.setPosition(0.8);
    //                })
    //                .UNSTABLE_addTemporalMarkerOffset(2, ()->{
    //                    lift.setPosition(0.46);
    //                })
    //                .UNSTABLE_addTemporalMarkerOffset(2.3, ()->{
    //                    movePlate(rotatePlate);
    //                })
    //                .UNSTABLE_addTemporalMarkerOffset(3.3, ()->{
    //                    lift.setPosition(0.8);
    //                })
    //                .UNSTABLE_addTemporalMarkerOffset(4.3, ()->{
    //                    lift.setPosition(0.46);
    //                })
    //                .UNSTABLE_addTemporalMarkerOffset(4.5, ()->{
    //                    movePlate(rotatePlate);
    //                })
    //                .UNSTABLE_addTemporalMarkerOffset(5.6, ()->{
    //                    lift.setPosition(0.8);
    //
    //                })
    //                .UNSTABLE_addTemporalMarkerOffset(5.9, ()->{
    //                    lift.setPosition(0.46);
    //                    shooter.setVelocity(0);
    //                    intake.setPower(1);
    //                })
    //                .waitSeconds(5.9)
    //
                    .waitSeconds(200)
                    .build();

            drive.setPoseEstimate(startingPose);
            limelight.start();
            thread.start();
            waitForStart();
            drive.followTrajectorySequence(DecodeAuto);
        }
    }
}