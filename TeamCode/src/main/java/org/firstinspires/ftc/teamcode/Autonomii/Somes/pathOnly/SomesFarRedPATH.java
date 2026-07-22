package org.firstinspires.ftc.teamcode.Autonomii.Somes.pathOnly;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Disabled
@Autonomous (name = "FarRedPATH", group = "Path")

public class SomesFarRedPATH extends LinearOpMode {
    Pose2d startingPose = new Pose2d(61, 11, 0); // Starts at red far facing red goal

    @Override
    public void runOpMode(){
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        TrajectorySequence exampleAuto = drive.trajectorySequenceBuilder(startingPose)

                // == PRELOAD (1) ==

                // Spline to shoot preload
                .setTangent(Math.toRadians(155))
                .splineToSplineHeading(new Pose2d(50, 16, Math.toRadians(-17)), Math.toRadians(155))

                // Give robot time to shoot
                .setTangent(Math.toRadians(70))
                .waitSeconds(3.4)

                // == HUMAN PLAYER (2) ==

                // Spline to human player (takes right-most artefact)
                .splineToSplineHeading(new Pose2d(55, 35.5, Math.toRadians(80)), Math.toRadians(70))
                .splineToConstantHeading(new Vector2d(58, 57), Math.toRadians(90))
                .setVelConstraint(new TranslationalVelocityConstraint(9))
                .splineToConstantHeading(new Vector2d(58, 60), Math.toRadians(90))

                // Third ball is NOT worth it </3

                // Spline to 2nd ball
                .setTangent(Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(61, 55), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(64, 60), Math.toRadians(90))
                .resetVelConstraint()

                // Spline to shooting position
                .setTangent(Math.toRadians(-105))
                .splineToConstantHeading(new Vector2d(59, 38), Math.toRadians(-100))
                .splineToSplineHeading(new Pose2d(55, 16, Math.toRadians(16)), Math.toRadians(-100))

                .setTangent(Math.toRadians(160))
                .waitSeconds(1.9) // give robot time to shoot

                // == THIRD SET (3) ==

                // Spline to set #3
                .setTangent(Math.toRadians(160))
                .splineToSplineHeading(new Pose2d(39, 20, Math.toRadians(65)), Math.toRadians(160))
                .splineToConstantHeading(new Vector2d(34, 22), Math.toRadians(90))
                .setVelConstraint(new TranslationalVelocityConstraint(11))
                .splineToConstantHeading(new Vector2d(34, 53), Math.toRadians(90))
                .resetVelConstraint()
                .splineToSplineHeading(new Pose2d(37, 63, Math.toRadians(90)), Math.toRadians(-65))

                // Spline to shoot
                .splineToSplineHeading(new Pose2d(55, 16, Math.toRadians(-9)), Math.toRadians(-65))

                .waitSeconds(100)

                .build();

        drive.setPoseEstimate(startingPose);

        waitForStart();
        drive.followTrajectorySequence(exampleAuto);
    }
}