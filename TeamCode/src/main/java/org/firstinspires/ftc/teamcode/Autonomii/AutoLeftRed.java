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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.List;

@Autonomous (name = "!AutoLeftRed")

public class AutoLeftRed extends LinearOpMode {
    public void movePlateID(int plateTarget, double power){
        plate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        plate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        plate.setPower(power);
        plate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        plate.setTargetPosition(plateTarget);
        plate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void movePlate(int plateTarget){
        plate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        plate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        plate.setPower(1);
        plate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        plate.setTargetPosition(plateTarget);
        plate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void movePlateGather(int plateTarget){
        plate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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
    Pose2d startingPose = new Pose2d(-51, -44, Math.toRadians(-125));

    class autoThread implements Runnable {
        @Override
        public void run() {

            while(opModeInInit() && !isStopRequested()){
                frontPlateServo.setPosition(0.5229);
                backPlateServo.setPosition(0.8729);
                servoShoot.setPosition(servoShootClose);
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
                .UNSTABLE_addTemporalMarkerOffset(0, ()->{
                    camera.setPosition(0.45);
                    shooter.setVelocity(motorRPM);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.3, ()->{
                    frontPlateServo.setPosition(0.8007);
                    backPlateServo.setPosition(0.1993);
                })
                .setTangent(Math.toRadians(-45))
                .splineToConstantHeading(new Vector2d(-18,8),Math.toRadians(-45))
                .UNSTABLE_addTemporalMarkerOffset(-0.1, ()->{
                    if (aprilTagID == 22) movePlateID(rotatePlate * 2, 1);
                    if (aprilTagID == 23) movePlateID(rotatePlate, 1);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.4, ()->{
                    firstAprilTag = true;
                    servoShoot.setPosition(servoShootOpen);
                })
                .UNSTABLE_addTemporalMarkerOffset(1, ()->{
                    lift.setPosition(liftUp);
                })
                .UNSTABLE_addTemporalMarkerOffset(2, ()->{
                    lift.setPosition(liftDown);
                })
                .UNSTABLE_addTemporalMarkerOffset(2.2, ()->{
                    movePlate(rotatePlate);
                })
                .UNSTABLE_addTemporalMarkerOffset(2.9, ()->{
                    lift.setPosition(liftUp);
                })
                .UNSTABLE_addTemporalMarkerOffset(3.3, ()->{
                    lift.setPosition(liftDown);
                })
                .UNSTABLE_addTemporalMarkerOffset(3.5, ()->{
                    movePlate(rotatePlate - 2);
                })
                .UNSTABLE_addTemporalMarkerOffset(4.6, ()->{
                    lift.setPosition(liftUp);
                })
                .UNSTABLE_addTemporalMarkerOffset(5, ()->{
                    lift.setPosition(liftDown);
                    shooter.setVelocity(0);
                    intake.setPower(1);
                })
                .UNSTABLE_addTemporalMarkerOffset(5.5, ()->{
                    servoShoot.setPosition(servoShootClose);
                    rotatePlate = 179;
                })
                .waitSeconds(5)
                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(-16, 8, Math.toRadians(90)), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-10, 26), Math.toRadians(90))
                .setVelConstraint(new TranslationalVelocityConstraint(17))
                .UNSTABLE_addTemporalMarkerOffset(0, ()->{
                    movePlateGather(rotatePlate);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.3, ()->{
                    movePlateGather(rotatePlate + 23);
                    shooter.setVelocity(motorRPM);
                })
                .splineToConstantHeading(new Vector2d(-10, 48), Math.toRadians(90))
                .resetVelConstraint()
                .UNSTABLE_addTemporalMarkerOffset(-0.2, ()->{
                    if(aprilTagID == 21) movePlateID(rotatePlate, 1);
                    if(aprilTagID == 23) movePlateID(rotatePlate*2, 1);
                })
                .setTangent(Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(-18, 8, Math.toRadians(125)), Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(0.3, ()->{
                    servoShoot.setPosition(servoShootOpen);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.6, ()->{
                    intake.setPower(0);
                    lift.setPosition(liftUp);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.2, ()->{
                    lift.setPosition(liftDown);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.4, ()->{
                    movePlate(rotatePlate);
                })
                .UNSTABLE_addTemporalMarkerOffset(2.5, ()->{
                    lift.setPosition(liftUp);
                })
                .UNSTABLE_addTemporalMarkerOffset(2.9, ()->{
                    lift.setPosition(liftDown);
                })
                .UNSTABLE_addTemporalMarkerOffset(3.1, ()->{
                    movePlate(rotatePlate);
                })
                .UNSTABLE_addTemporalMarkerOffset(4.2, ()->{
                    lift.setPosition(liftUp);
                })
                .UNSTABLE_addTemporalMarkerOffset(4.6, ()->{
                    lift.setPosition(liftDown);
                    shooter.setVelocity(0);
                    intake.setPower(1);
                })
                .UNSTABLE_addTemporalMarkerOffset(5.1, ()->{
                    servoShoot.setPosition(servoShootClose);
                })
                .waitSeconds(4.6)
                .setTangent(0)
                .splineToSplineHeading(new Pose2d(5, 14, Math.toRadians(110)), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(12, 14), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(17.4, 23), Math.toRadians(90))
                .setVelConstraint(new TranslationalVelocityConstraint(17))
                .UNSTABLE_addTemporalMarkerOffset(0.2, ()->{
                    movePlateGather(rotatePlate);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.5, ()->{
                    movePlateGather(rotatePlate);
                    shooter.setVelocity(motorRPM);
                })
                .splineToConstantHeading(new Vector2d(17.4, 54), Math.toRadians(90))
                .resetVelConstraint()
                .UNSTABLE_addTemporalMarkerOffset(0.4, ()->{
                    if(aprilTagID == 22) movePlateID((rotatePlate*2)+23, 1);
                    if(aprilTagID == 23) movePlateID(rotatePlate + 23, 1);
                })
                .setTangent(Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(-20, 8, Math.toRadians(125)), Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(0.7, ()->{
                    servoShoot.setPosition(servoShootOpen);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.1, ()->{
                    intake.setPower(0);
                    lift.setPosition(liftUp);
                    shooter.setVelocity(1075);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.6, ()->{
                    lift.setPosition(liftDown);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.8, ()->{
                    movePlate(rotatePlate);
                    shooter.setVelocity(motorRPM);
                })
                .UNSTABLE_addTemporalMarkerOffset(2.2, ()->{
                    lift.setPosition(liftUp);
                })
                .UNSTABLE_addTemporalMarkerOffset(3.4, ()->{
                    lift.setPosition(liftDown);
                })
                .UNSTABLE_addTemporalMarkerOffset(3.8, ()->{
                    movePlate(rotatePlate);
                })
                .UNSTABLE_addTemporalMarkerOffset(4.9, ()->{
                    lift.setPosition(liftUp);
                })
                .UNSTABLE_addTemporalMarkerOffset(5.3, ()->{
                    lift.setPosition(liftDown);
                    shooter.setVelocity(0);
                })
                .waitSeconds(200)
                .build();

        drive.setPoseEstimate(startingPose);
        limelight.start();
        thread.start();
        waitForStart();
        drive.followTrajectorySequence(DecodeAuto);
    }
}