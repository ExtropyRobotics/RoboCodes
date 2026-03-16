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
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.List;
@Disabled
@Autonomous (name = "!AutoLeftBLUE")

public class AutoLeftBlue extends LinearOpMode {
    double flywheelF = 0;
    double flywheelP = 0;
    DcMotorEx flywheelMotor;

    public void turretObeliskStop(){

    }
    public void sensorStart() {

    }

    public void getGoalDistance(){

    }

    public void turretStart() {

    }

    public void getPlatePosition(){
        // Average encoder method
    }

    public void plateCalculations(){

    }
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
    Pose2d startingPose = new Pose2d(-51, -44, Math.toRadians(-125));


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
                plateCalculations();
                llResult = limelight.getLatestResult();

                turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                if (llResult != null && llResult.isValid()) {
                    limelight.pipelineSwitch(0);
                    List<LLResultTypes.FiducialResult> fiducialResults = llResult.getFiducialResults();
                    for (LLResultTypes.FiducialResult fr : fiducialResults) {
                        telemetry.addData("ID", fr.getFiducialId());
                        aprilTagID = fr.getFiducialId();
                        telemetry.update();
                    }
                }
                if(aprilTagID != 0){
                    turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    turret.setTargetPosition(600);
                    limelight.pipelineSwitch(1);
                    turretObeliskStop();
                    turretStart();
                }

                PIDFCoefficients pidfCoefficients
                = new PIDFCoefficients(flywheelP, 0, 0, flywheelF);
                flywheelMotor.setPIDFCoefficients(DcMotor.
                        RunMode.RUN_USING_ENCODER, pidfCoefficients);
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
                .setTangent(Math.toRadians(45))
                .splineToConstantHeading(new Vector2d(-18,-8),Math.toRadians(45))
                .UNSTABLE_addTemporalMarkerOffset(1, ()->{
                    movePlate(237, 1);
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