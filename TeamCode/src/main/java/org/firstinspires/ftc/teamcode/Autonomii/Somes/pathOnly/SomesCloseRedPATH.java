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

@Autonomous (name = "CloseRedPATH", group = "Path")

public class SomesCloseRedPATH extends LinearOpMode {
    Pose2d startingPose = new Pose2d(-50, 48, Math.toRadians(-60)); // Starts at red goal facing red goal

    @Override
    public void runOpMode(){
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        TrajectorySequence exampleAuto = drive.trajectorySequenceBuilder(startingPose)

                // == PRELOAD (1) ==

                // Spline to first shooting position
                .setTangent(Math.toRadians(-45))
                .splineToSplineHeading(new Pose2d(-18, 18, Math.toRadians(-50)), Math.toRadians(-45))

                // Give robot time to shoot
                .waitSeconds(1.2)

                // == FIRST SET (2) ==

                // Spline to the first set
                .splineToSplineHeading(new Pose2d(-13, 19, Math.toRadians(70)), Math.toRadians(90))
                .setVelConstraint(new TranslationalVelocityConstraint(13))
                .splineToConstantHeading(new Vector2d(-10, 30), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-10, 45), Math.toRadians(90))
                .resetVelConstraint()
                .splineToSplineHeading(new Pose2d(-13, 50, Math.toRadians(90)), Math.toRadians(-90))

                // Spline to shooting position.
                .splineToSplineHeading(new Pose2d(-20, 18, Math.toRadians(-45)), Math.toRadians(-110))

                // Give robot time to shoot
                .waitSeconds(1.2)

                // == SECOND SET (3) ==

                // Spline to second set
                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(4, 17, Math.toRadians(70)), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(8, 17), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(13, 20), Math.toRadians(90))
                .setVelConstraint(new TranslationalVelocityConstraint(13))
                .splineToConstantHeading(new Vector2d(15.5, 53), Math.toRadians(90))
                .resetVelConstraint()
                .splineToSplineHeading(new Pose2d(14, 62, Math.toRadians(-90)), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(14, 50), Math.toRadians(-90))

                // Spline to shooting position
                .splineToSplineHeading(new Pose2d(-20, 18, Math.toRadians(-35)), Math.toRadians(-125))

                // Give robot time to shoot
                .waitSeconds(1.2)

                // == THIRD SET (4) ==

                // Spline to third set
                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(29, 25, Math.toRadians(70)), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(34, 23), Math.toRadians(90))
                .setVelConstraint(new TranslationalVelocityConstraint(13))
                .splineToConstantHeading(new Vector2d(34, 53), Math.toRadians(90))
                .resetVelConstraint()
                .splineToSplineHeading(new Pose2d(34, 63, Math.toRadians(90)), Math.toRadians(-90))

                // Spline to open gate
                .splineToSplineHeading(new Pose2d(2, 48, Math.toRadians(0)), Math.toRadians(90))

                // Spline to shooting position
                .splineToSplineHeading(new Pose2d(-20, 18, Math.toRadians(-25)), Math.toRadians(-100))

                .waitSeconds(100)

                .build();

        drive.setPoseEstimate(startingPose);

        waitForStart();
        drive.followTrajectorySequence(exampleAuto);
    }
}