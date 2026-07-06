package org.firstinspires.ftc.teamcode.Autonomii.Somes.pathOnly;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous (name = "FarBluePATH", group = "Path")

public class SomesFarBluePATH extends LinearOpMode {
    Pose2d startingPose = new Pose2d(61, -8, 0); // Starts at blue far facing blue goal

    @Override
    public void runOpMode(){
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        TrajectorySequence exampleAuto = drive.trajectorySequenceBuilder(startingPose)

                // == PRELOAD (1) ==

                // Spline to shoot preload
                .setTangent(Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(55, -10, Math.toRadians(31)), Math.toRadians(-120))

                // Give robot time to shoot
                .waitSeconds(3)

                // == HUMAN PLAYER (2) ==

                // Spline to human player
                .setTangent(Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(55, -50, Math.toRadians(-75)), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(55, -56), Math.toRadians(-90))
                .setTangent(Math.toRadians(0))
                .setVelConstraint(new TranslationalVelocityConstraint(10))
                .splineToConstantHeading(new Vector2d(60, -56), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(61, -56, Math.toRadians(-90)), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(63, -63), Math.toRadians(-90))
                .resetVelConstraint()

                // Spline to shooting position
                .setTangent(Math.toRadians(110))
                .splineToSplineHeading(new Pose2d(55, -40, Math.toRadians(31)), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(55, -10), Math.toRadians(90))

                // Give robot time to shoot
                .waitSeconds(3)

                // == THIRD SET (3) ==

                // Spline to set #3
                .setTangent(Math.toRadians(-145))
                .splineToSplineHeading(new Pose2d(36, -23, Math.toRadians(-70)), Math.toRadians(-145))
                .setVelConstraint(new TranslationalVelocityConstraint(13))
                .splineToConstantHeading(new Vector2d(34, -25), Math.toRadians(-90))
                .setVelConstraint(new TranslationalVelocityConstraint(13))
                .splineToConstantHeading(new Vector2d(34, -53), Math.toRadians(-90))
                .resetVelConstraint()
                .splineToSplineHeading(new Pose2d(34, -63, Math.toRadians(-90)), Math.toRadians(90))

                // Spline to shooting position
                .setTangent(Math.toRadians(70))
                .splineToSplineHeading(new Pose2d(55, -10, Math.toRadians(31)), Math.toRadians(70))

                .build();

        drive.setPoseEstimate(startingPose);

        waitForStart();
        drive.followTrajectorySequence(exampleAuto);
    }
}