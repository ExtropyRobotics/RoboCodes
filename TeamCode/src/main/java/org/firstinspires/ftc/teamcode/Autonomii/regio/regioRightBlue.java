//package org.firstinspires.ftc.teamcode.Autonomii;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.geometry.Vector2d;
//import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;
//import com.qualcomm.hardware.limelightvision.LLResult;
//import com.qualcomm.hardware.limelightvision.LLResultTypes;
//import com.qualcomm.hardware.limelightvision.Limelight3A;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.DistanceSensor;
//import com.qualcomm.robotcore.hardware.IMU;
//import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
//import com.qualcomm.robotcore.hardware.NormalizedRGBA;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.TeleOPs.newRobot.regioTeleOP;
//import org.firstinspires.ftc.teamcode.TeleOPs.newRobot.twoMotorsOuttakeTest;
//import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
//import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
//import org.firstinspires.ftc.teamcode.util.Encoder;
//
//import java.util.List;
//@Disabled
//@Autonomous (name = "!regioRightBLUE")
//
//public class regioRightBlue extends LinearOpMode {
//    public enum StateList {
//        Intake,
//        Rotate,
//        Outtake
//    }
//    volatile StateList state = StateList.Intake;
//    public static class Artefact{
//        volatile int pos = 0;
//        volatile int color = 0;
//    }
//
//    volatile regioTeleOP.Artefact[] storage = {new regioTeleOP.Artefact(), new regioTeleOP.Artefact(), new regioTeleOP.Artefact()};
//
//
//    // PID
//    volatile private double kP = 0.02;
//    volatile private double kI = 0.0;
//    volatile private double kD = 0.0;
//    volatile private double kF = 0.0;
//
//    volatile private double turretIntegral = 0;
//    volatile private double turretLastError = 0;
//    volatile private double turretLastOutput = 0;
//    volatile double error = 0;
//    volatile private ElapsedTime timer = new ElapsedTime();
//    // limits
//    volatile int MAX_TURRET_TICKS = 2100;
//    volatile int MIN_TURRET_TICKS = -230;
//    volatile private static double POSITION_TOLERANCE = 1.5;
//    volatile private static double MAX_POWER = 1;
//    volatile private static  double MAX_OUTPUT_CHANGE = 0.05;
//    volatile private static  double INTEGRAL_LIMIT = 10;
//    volatile int currentPos = 0;
//    volatile double maxPlatePower = 1;
//
//    // memory
//    volatile double tx = 0;
//    volatile private double lastTx = 0;
//    volatile private double lastTxVelocity = 0;
//    volatile double searchPower = 0;
//    // hardware
//    volatile DcMotorEx turret;
//    volatile Limelight3A limelight;
//    volatile SampleMecanumDrive drive;
//    volatile boolean unwinding = false;
//    volatile private boolean tagVisible = true;
//
//    // hardware
//    volatile Encoder plateEncoder;
//    volatile CRServo plateServoLeft;
//    volatile CRServo plateServoRight;
//    volatile DcMotorEx intake;
//    volatile DcMotorEx outtake;
//    volatile DcMotorEx outtake2;
//    volatile NormalizedColorSensor colorSensor;
//    volatile DistanceSensor distanceSensor;
//    volatile IMU imu;
//
//    volatile int plateTargetPosition = 0;
//    volatile int i = 2;
//    volatile int pattern = 0;
//
//    volatile int sumPlate = 0;
//    volatile int encoderPos = 0;
//    volatile double plateError = 0;
//
//    volatile double platePow = 1;
//    volatile double plateP = 0.0002;
//    volatile double plateI = 0.00000;
//    volatile double plateD = 0.0008;
//
//    volatile double artefactDistance = 0;
//    volatile NormalizedRGBA colors;
//
//    volatile ElapsedTime i2cTimer = new ElapsedTime();
//
//    volatile int artefactsGathered = -1;
//    volatile boolean full = false;
//    volatile boolean plateAtPosition = false;
//    volatile boolean artefactTaken = false;
//    volatile boolean shootFound = false;
//    volatile boolean shooterReady = false;
//    volatile boolean atPattern = false;
//
//    volatile boolean pattern1Toggle = false;
//    volatile boolean pattern2Toggle = false;
//    volatile boolean pattern3Toggle = false;
//    volatile boolean manualPlateMove = false;
//    volatile boolean ballOneShot = false;
//    volatile boolean ballTwoShot = false;
//    volatile boolean ballThreeShot = false;
//    volatile double dt;
//
//    volatile ElapsedTime outtakeStop = new ElapsedTime();
//    volatile ElapsedTime shootCooldown = new ElapsedTime();
//    volatile Pose2d startingPose = new Pose2d(-51, -44, Math.toRadians(-125));
//
//
//    class autoThread implements Runnable {
//        @Override
//        public void run() {
//
//            while (opModeIsActive() && !isStopRequested()) {
//                dt = timer.seconds();
//
//                plateAtPosition = plateTargetPosition < plateEncoder.getCurrentPosition() + 200 && plateTargetPosition > plateEncoder.getCurrentPosition() - 200;
//
//                double plateDt = timer.seconds();
//
//                encoderPos = plateEncoder.getCurrentPosition();
//                plateError = plateTargetPosition - encoderPos;
//                sumPlate += (int) (plateError * plateDt);
//                double deriv = (plateTargetPosition - encoderPos - plateError);
//
//                platePow = plateP*plateError + plateI*sumPlate + deriv* plateD;
//
//                double dt = timer.seconds();
//                currentPos = turret.getCurrentPosition();
//
//                lastTxVelocity = (error - lastTx) / dt;
//                searchPower =  0.4 * Math.signum(error);
//                if (dt <= 0) dt = 0.02;
//
//                Pose2d poseVelocity = drive.getPoseVelocity();
//                double robotOmega = 0;
//                if (poseVelocity != null) {
//                    robotOmega = poseVelocity.getHeading();
//                }
//
//                if (unwinding) {
//                    int error = -currentPos;
//
//                    if (Math.abs(error) < 200) {
//                        turret.setPower(0);
//                        unwinding = false;
//                        turretIntegral = 0;
//                        turretLastError = 0;
//                    } else {
//                        double power = 0.4 * Math.signum(error);
//                        turret.setPower(power);
//                    }
//                    return;
//                }
//
//                if (currentPos >= MAX_TURRET_TICKS || currentPos <= MIN_TURRET_TICKS) {
//                    unwinding = true;
//                    return;
//                }
//
//                // limelight init
//                LLResult result = limelight.getLatestResult();
//                boolean currentlyVisible = false;
//
//                if (result != null && result.isValid()) {
//                    List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
//                    if (tags != null && !tags.isEmpty()) {
//                        tx = tags.get(0).getTargetXDegrees();
//                        currentlyVisible = true;
//                    }
//                }
//
//                if (currentlyVisible && !unwinding) {
//                    // normal PID control
//                    timer.reset();
//                    // update vel for future reacquisition
//                    error = tx;
//
//                    if (Math.abs(error) < POSITION_TOLERANCE) {
//                        turret.setPower(0);
//                        turretIntegral = 0;
//                        turretLastError = 0;
//                        turretLastOutput = 0;
//                    } else {
//                        turretIntegral += error * dt;
//                        turretIntegral = Math.max(-INTEGRAL_LIMIT, Math.min(INTEGRAL_LIMIT, turretIntegral));
//                        double derivative = (error - turretLastError) / dt;
//
//                        double output = (kP * error) + (kI * turretIntegral) + (kD * derivative) + (kF * robotOmega);
//
//                        // clamp output
//                        output = Math.max(-MAX_POWER, Math.min(MAX_POWER, output));
//
//                        double delta = output - turretLastOutput;
//                        delta = Math.max(-MAX_OUTPUT_CHANGE, Math.min(MAX_OUTPUT_CHANGE, delta));
//                        output = turretLastOutput + delta;
//
//                        turret.setPower(output);
//
//                        turretLastError = error;
//                        turretLastOutput = output;
//                        lastTx = tx;
//                    }
//
//                }
//                if (!currentlyVisible && !unwinding){
//                    // continuous movement opposite the last observed tag motion
//                    turret.setPower(searchPower);
//                }
//
//                switch (state){
//                    case Intake:
//
//                        if(artefactsGathered == -1) plateTargetPosition = 0;
//
//                        if(plateAtPosition){
//                            if(artefactDistance < 100 && !full){
//                                if(!artefactTaken){
//                                    artefactsGathered += 1;
//                                    state = StateList.Rotate;
//                                    artefactTaken = true;
//                                } else artefactTaken = false;
//                            }
//                        }
//
//                        if(outtakeStop.seconds() > 5){
//                            outtake.setPower(0);
//                            outtake2.setPower(0);
//                        }
//
//                        shootCooldown.reset();
//
//                        break;
//
//                    case Rotate:
//
//                        storage[artefactsGathered].pos = plateEncoder.getCurrentPosition();
//
//                        break;
//
//                    case Outtake:
//                        outtake.setPower(1);
//                        outtake2.setPower(1);
//
//                        if(outtake.getVelocity() > 1700) {
//                            shooterReady = true;
//                        } else shooterReady = false;
//
//                        if(shooterReady && plateAtPosition && atPattern && !ballOneShot) {
//                            plateTargetPosition -= 8192/3;
//
//                            storage[0].pos = 0;
//                            shootCooldown.reset();
//                            ballOneShot = true;
//                            full = false;
//                        }
//
//                        if(ballOneShot && !ballTwoShot && shootCooldown.seconds() >= 1){
//                            plateTargetPosition -= 8192/3;
//
//                            storage[1].pos = 0;
//
//                            shootCooldown.reset();
//                            ballTwoShot = true;
//                        }
//
//                        if(ballTwoShot && !ballThreeShot && shootCooldown.seconds() >= 1){
//                            plateTargetPosition -= 8192/3;
//
//                            storage[2].pos = 0;
//                            ballThreeShot = true;
//                            artefactsGathered = -1;
//                        }
//
//                        if(ballThreeShot){
//                            state = StateList.Intake;
//                            ballOneShot = false;
//                            ballTwoShot = false;
//                            ballThreeShot = false;
//                            shooterReady = false;
//                            shootFound = false;
//                            pattern = 0;
//                            atPattern = false;
//                            outtakeStop.reset();
//                        }
//
//                        break;
//                }
//            }
//        }
//    }
//
//    autoThread obj = new autoThread();
//
//    @Override
//    public void runOpMode(){
//        Thread thread = new Thread(obj);
//
//        drive = new SampleMecanumDrive(hardwareMap);
//
//        plateServoLeft = hardwareMap.get(CRServo.class, "plateServoLeft");
//        plateServoRight = hardwareMap.get(CRServo.class, "plateServoRight");
//
//        intake = hardwareMap.get(DcMotorEx.class, "intake");
//        outtake = hardwareMap.get(DcMotorEx.class, "outtake");
//        turret = hardwareMap.get(DcMotorEx.class, "turret");
//
//        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");
//        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
//
//        imu = hardwareMap.get(IMU.class, "imu");
//
//        plateEncoder = hardwareMap.get(Encoder.class, "outtake2");
//        plateEncoder.setDirection(Encoder.Direction.REVERSE);
//
//        limelight = hardwareMap.get(Limelight3A.class, "limelight");
//
//        outtake2 = hardwareMap.get(DcMotorEx.class, "outtake2");
//
//        outtake.setDirection(DcMotorSimple.Direction.REVERSE);
//        outtake2.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        // TODO: Set your AprilTag pipeline here before each match
//        // 1 - Blue alliance
//        // 2 - Red alliance
//        limelight.pipelineSwitch(1);
//        limelight.start();
//
//        i2cTimer.reset();
//
//        TrajectorySequence DecodeAuto= drive.trajectorySequenceBuilder(startingPose)
//                .splineToSplineHeading(new Pose2d(50, -15, Math.toRadians(-155)), Math.toRadians(210))
//                .UNSTABLE_addTemporalMarkerOffset(0, ()->{
//                    state = StateList.Outtake;
//                })
//                .waitSeconds(5)
//                .splineToSplineHeading(new Pose2d(37, -32.5, Math.toRadians(-90)), Math.toRadians(-90))
//                .setVelConstraint(new TranslationalVelocityConstraint(30))
//                .splineToConstantHeading(new Vector2d(37, -49), Math.toRadians(70))
//                .resetVelConstraint()
//                .splineToSplineHeading(new Pose2d(43.5, -32, Math.toRadians(-155)), Math.toRadians(70))
//                .splineToConstantHeading(new Vector2d(50, -15), Math.toRadians(70))
//                .setTangent(Math.toRadians(-160))
//                .UNSTABLE_addTemporalMarkerOffset(5, ()->{
//                    state = StateList.Outtake;
//                })
//                .waitSeconds(5)
//                .splineToSplineHeading(new Pose2d(25, -22, Math.toRadians(-90)), Math.toRadians(-160))
//                .splineToConstantHeading(new Vector2d(11, -32.5), Math.toRadians(-90))
//                .setVelConstraint(new TranslationalVelocityConstraint(30))
//                .splineToConstantHeading(new Vector2d(11, -49), Math.toRadians(50))
//                .resetVelConstraint()
//                .splineToSplineHeading(new Pose2d(50, -15, Math.toRadians(-155)), Math.toRadians(50))
//                .setTangent(Math.toRadians(-170))
//                .UNSTABLE_addTemporalMarkerOffset(5, ()->{
//                    state = StateList.Outtake;
//                })
//                .waitSeconds(200)
//                .build();
//
//        drive.setPoseEstimate(startingPose);
//        limelight.start();
//
//        thread.start();
//        waitForStart();
//        timer.reset();
//        outtakeStop.reset();
//        double dt = timer.seconds();
//        drive.followTrajectorySequence(DecodeAuto);
//    }
//}