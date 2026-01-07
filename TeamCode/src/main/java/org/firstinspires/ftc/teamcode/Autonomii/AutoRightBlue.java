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

@Autonomous (name = "!AutoRightBlue")

public class AutoRightBlue extends LinearOpMode {
    public void movePlateID(int plateTarget){
        plate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        plate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        plate.setPower(0.35);
        plate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        plate.setTargetPosition(plateTarget);
        plate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void movePlate(int plateTarget){
        plate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        plate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        plate.setPower(0.45);
        plate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        plate.setTargetPosition(plateTarget);
        plate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    DcMotor plate = null;
    DcMotor intake = null;
    DcMotorEx shooter = null;
    Servo lift = null;
    Servo camera = null;
    Limelight3A limelight;
    LLResult llResult;
    int aprilTagID = 0;
    NormalizedColorSensor colorSensor;
    DistanceSensor distanceSensor;
    Pose2d startingPose = new Pose2d(58, -10, Math.toRadians(180));
    int rotatePlate = 176;

    class autoThread implements Runnable{
        @Override
        public void run() {
            while(opModeIsActive() && !isStopRequested()){
                llResult = limelight.getLatestResult();

                if(llResult != null && llResult.isValid()) {
                    limelight.pipelineSwitch(0);
                    List<LLResultTypes.FiducialResult> fiducialResults = llResult.getFiducialResults();
                    for (LLResultTypes.FiducialResult fr : fiducialResults) {
                        telemetry.addData("ID", fr.getFiducialId());
                        aprilTagID = fr.getFiducialId();
                        telemetry.update();
                    }
                }
                camera.setPosition(0.76);
                telemetry.addData("aa", 1);
                telemetry.update();
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
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distance");
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "color");

        TrajectorySequence DecodeAuto= drive.trajectorySequenceBuilder(startingPose)
                .setTangent(Math.toRadians(-115))
                .UNSTABLE_addTemporalMarkerOffset(0, ()->{
                    shooter.setVelocity(1350);
                    camera.setPosition(0.76);
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
                    movePlate(rotatePlate);
                })
                .UNSTABLE_addTemporalMarkerOffset(3.7, ()->{
                    lift.setPosition(0.8);
                })
                .UNSTABLE_addTemporalMarkerOffset(4.2, ()->{
                    lift.setPosition(0.46);
                })
                .UNSTABLE_addTemporalMarkerOffset(4.4, ()->{
                    movePlate(rotatePlate);
                })
                .UNSTABLE_addTemporalMarkerOffset(5.4, ()->{
                    lift.setPosition(0.8);
                })
                .UNSTABLE_addTemporalMarkerOffset(5.9, ()->{
                    lift.setPosition(0.46);
                    shooter.setVelocity(0);
                    intake.setPower(1);
                })
                .waitSeconds(5.6)
                .setTangent(180)
                .splineToSplineHeading(new Pose2d(36, -25, Math.toRadians(-90)), Math.toRadians(-90))
                .setVelConstraint(new TranslationalVelocityConstraint(18))
                .UNSTABLE_addTemporalMarkerOffset(0.6, ()->{
                    movePlateID(rotatePlate);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.1, ()->{
                    movePlateID(rotatePlate);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.7, ()->{
                    shooter.setVelocity(1000);
                })
                .UNSTABLE_addTemporalMarkerOffset(2.2, ()->{
                    if(aprilTagID == 21) movePlateID(rotatePlate*2);
                    if(aprilTagID == 22) movePlateID(rotatePlate);
                })
                .UNSTABLE_addTemporalMarkerOffset(2.8, ()->{
                    intake.setPower(0);
                    shooter.setVelocity(1350);
                })
                .splineToConstantHeading(new Vector2d(36, -60), Math.toRadians(-90))
                .resetVelConstraint()
                .setTangent(Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(50, -15, Math.toRadians(-155)), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(1, ()->{
                    lift.setPosition(0.8);
                })
                .UNSTABLE_addTemporalMarkerOffset(2, ()->{
                    lift.setPosition(0.46);
                })
                .UNSTABLE_addTemporalMarkerOffset(2.3, ()->{
                    movePlate(rotatePlate);
                })
                .UNSTABLE_addTemporalMarkerOffset(3.3, ()->{
                    lift.setPosition(0.8);
                })
                .UNSTABLE_addTemporalMarkerOffset(4.3, ()->{
                    lift.setPosition(0.46);
                })
                .UNSTABLE_addTemporalMarkerOffset(4.5, ()->{
                    movePlate(rotatePlate);
                })
                .UNSTABLE_addTemporalMarkerOffset(5.6, ()->{
                    lift.setPosition(0.8);
                })
                .UNSTABLE_addTemporalMarkerOffset(5.9, ()->{
                    lift.setPosition(0.46);
                    shooter.setVelocity(0);
                    intake.setPower(1);
                })
                .waitSeconds(5.9)

                .waitSeconds(200)
                .build();

        drive.setPoseEstimate(startingPose);
        limelight.start();
        thread.start();
        waitForStart();
        sleep(500);
        drive.followTrajectorySequence(DecodeAuto);
    }
}