package org.firstinspires.ftc.teamcode.Autonomii;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.List;
@Disabled
@Autonomous (name = "!AutoRightBlue")

public class AutoRightBlue extends LinearOpMode {
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