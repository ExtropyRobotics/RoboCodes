package org.firstinspires.ftc.teamcode.Autonomii;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


@Autonomous (name = "!DecodeAuto")

public class decodeAuto extends LinearOpMode {

    Pose2d startingPose = new Pose2d(0, 0,Math.toRadians(90));

    @Override
    public void runOpMode(){
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(),telemetry);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        TrajectorySequence DecodeAuto= drive.trajectorySequenceBuilder(startingPose)
                .UNSTABLE_addTemporalMarkerOffset(0, ()->{
                })
                .setTangent(Math.toRadians(160))
                .splineToSplineHeading(new Pose2d(-59, -56, Math.toRadians(45)), Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(0.4, ()->{

                })
                .waitSeconds(5.1)
                .setTangent(Math.toRadians(45))
                .splineToSplineHeading(new Pose2d(-55, -10, Math.toRadians(90)), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-56.5, -10), Math.toRadians(180))
                .resetVelConstraint()
                .splineToConstantHeading(new Vector2d(-56.5, -58), Math.toRadians(270))
                .waitSeconds(200)
                .build();

        drive.setPoseEstimate(startingPose);
        waitForStart();
        sleep(500);
        drive.followTrajectorySequence(DecodeAuto);

    }
}