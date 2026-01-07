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
    public void movePlate(int plateTarget){
        plate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        plate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        plate.setPower(0.8);
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
    Pose2d startingPose = new Pose2d(-51, 44, Math.toRadians(-125));

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
                .UNSTABLE_addTemporalMarkerOffset(0, ()->{
                            shooter.setVelocity(1000);
                            if(aprilTagID == 22) movePlate(179*2);
                            if(aprilTagID == 23) movePlate(179);
                        })
                        .setTangent(Math.toRadians(-45))
                            .splineToConstantHeading(new Vector2d(-31,21),Math.toRadians(-45))
                        .UNSTABLE_addTemporalMarkerOffset(1, ()->{
                            lift.setPosition(0.8);
                            shooter.setVelocity(1100);
                        })
                        .UNSTABLE_addTemporalMarkerOffset(1.5, ()->{
                            lift.setPosition(0.46);
                        })
                        .UNSTABLE_addTemporalMarkerOffset(1.7, ()->{
                            movePlate(179);
                        })
                        .UNSTABLE_addTemporalMarkerOffset(2.7, ()->{
                            lift.setPosition(0.8);
//                    shooter.setVelocity(1100);
                        })
                        .UNSTABLE_addTemporalMarkerOffset(3.2, ()->{
                            lift.setPosition(0.46);
                        })
                        .UNSTABLE_addTemporalMarkerOffset(3.4, ()->{
                            movePlate(179);
                        })
                        .UNSTABLE_addTemporalMarkerOffset(4.4, ()->{
                            lift.setPosition(0.8);
                        })
                        .UNSTABLE_addTemporalMarkerOffset(4.9, ()->{
                            lift.setPosition(0.46);
                            shooter.setVelocity(0);
                            intake.setPower(1);
                        })
                            .waitSeconds(4.9)
                            .setTangent(Math.toRadians(-45))
                            .splineToSplineHeading(new Pose2d(-13, 18, Math.toRadians(90)), Math.toRadians(0))
                            .splineToConstantHeading(new Vector2d(-8, 26), Math.toRadians(90))
                            .setVelConstraint(new TranslationalVelocityConstraint(10))
                        .UNSTABLE_addTemporalMarkerOffset(0.4, ()->{
                            movePlate(179);
                        })
                        .UNSTABLE_addTemporalMarkerOffset(1.2, ()->{
                            movePlate(179);
                        })
                        .UNSTABLE_addTemporalMarkerOffset(1.6, ()->{
                            shooter.setVelocity(1000);
                        })
                        .UNSTABLE_addTemporalMarkerOffset(2.1, ()->{
                            intake.setPower(0);
                            if(aprilTagID == 21) movePlate(179);
                            if(aprilTagID == 23) movePlate(179*2);
                        })
                            .splineToConstantHeading(new Vector2d(-8, 45), Math.toRadians(90))
                            .resetVelConstraint()
                            .setTangent(Math.toRadians(-90))
                            .splineToSplineHeading(new Pose2d(-30, 20, Math.toRadians(120)), Math.toRadians(180))
                        .UNSTABLE_addTemporalMarkerOffset(0.6, ()->{
                            lift.setPosition(0.8);
                            shooter.setVelocity(1000);
                        })
                        .UNSTABLE_addTemporalMarkerOffset(1.1, ()->{
                            lift.setPosition(0.46);
                        })
                        .UNSTABLE_addTemporalMarkerOffset(1.3, ()->{
                            movePlate(179);
                        })
                        .UNSTABLE_addTemporalMarkerOffset(2.2, ()->{
                            lift.setPosition(0.8);
//                    shooter.setVelocity(1100);
                        })
                        .UNSTABLE_addTemporalMarkerOffset(2.8, ()->{
                            lift.setPosition(0.46);
                        })
                        .UNSTABLE_addTemporalMarkerOffset(3, ()->{
                            movePlate(179);
                        })
                        .UNSTABLE_addTemporalMarkerOffset(4, ()->{
                            lift.setPosition(0.8);
                        })
                        .UNSTABLE_addTemporalMarkerOffset(4.5, ()->{
                            lift.setPosition(0.46);
                            shooter.setVelocity(0);
                            intake.setPower(1);
                        })
                            .waitSeconds(4.5)
                            .setTangent(Math.toRadians(0))
                            .splineToSplineHeading(new Pose2d(5, 18, Math.toRadians(90)), Math.toRadians(0))
                            .splineToConstantHeading(new Vector2d(13, 26), Math.toRadians(90))
                            .setVelConstraint(new TranslationalVelocityConstraint(10))
                        .UNSTABLE_addTemporalMarkerOffset(0.7, ()->{
                            movePlate(179);
                        })
                        .UNSTABLE_addTemporalMarkerOffset(1.5, ()->{
                            movePlate(179);
                        })
                        .UNSTABLE_addTemporalMarkerOffset(2.1, ()->{
                            shooter.setVelocity(1000);
                        })
                        .UNSTABLE_addTemporalMarkerOffset(2.2, ()->{
                            intake.setPower(0);
                            if(aprilTagID == 22) movePlate(179*2);
                            if(aprilTagID == 23) movePlate(179);
                        })
                            .splineToConstantHeading(new Vector2d(13, 45), Math.toRadians(-90))
                            .resetVelConstraint()
                            .setTangent(Math.toRadians(-90))
                            .splineToSplineHeading(new Pose2d(-30, 20, Math.toRadians(120)), Math.toRadians(180))
                        .UNSTABLE_addTemporalMarkerOffset(1, ()->{
                            lift.setPosition(0.8);
                            shooter.setVelocity(1000);
                        })
                        .UNSTABLE_addTemporalMarkerOffset(1.5, ()->{
                            lift.setPosition(0.46);
                        })
                        .UNSTABLE_addTemporalMarkerOffset(1.7, ()->{
                            movePlate(179);
                        })
                        .UNSTABLE_addTemporalMarkerOffset(2.6, ()->{
                            lift.setPosition(0.8);
//                    shooter.setVelocity(1100);
                        })
                        .UNSTABLE_addTemporalMarkerOffset(3.2, ()->{
                            lift.setPosition(0.46);
                        })
                        .UNSTABLE_addTemporalMarkerOffset(3.4, ()->{
                            movePlate(179);
                        })
                        .UNSTABLE_addTemporalMarkerOffset(4.4, ()->{
                            lift.setPosition(0.8);
                        })
                        .UNSTABLE_addTemporalMarkerOffset(4.8, ()->{
                            lift.setPosition(0.46);
                            shooter.setVelocity(0);
                            intake.setPower(0);
                        })
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