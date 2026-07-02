package org.firstinspires.ftc.teamcode.Autonomii.Somes.pathOnly;

import static java.lang.Math.signum;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous (name = "CloseBluePATH", group = "Path")

public class SomesCloseBluePATH extends LinearOpMode {
    Pose2d startingPose = new Pose2d(-50, -48, Math.toRadians(60)); // Starts at blue goal facing blue goal

    @Override
    public void runOpMode(){

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        TrajectorySequence autonomous = drive.trajectorySequenceBuilder(startingPose)
                .setTangent(Math.toRadians(45))
                .splineToSplineHeading(new Pose2d(-25, -25, Math.toRadians(40)), Math.toRadians(45))

                .waitSeconds(1.25)

                .splineToSplineHeading(new Pose2d(-12, -25, Math.toRadians(-70)), Math.toRadians(-90))
                .setVelConstraint(new TranslationalVelocityConstraint(13))
                .splineToConstantHeading(new Vector2d(-12, -30), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(-12, -48), Math.toRadians(-90))
                .resetVelConstraint()


                .setTangent(Math.toRadians(110))
                .splineToSplineHeading(new Pose2d(-20, -25, Math.toRadians(40)), Math.toRadians(110))

                .waitSeconds(1.25)

                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(8, -25, Math.toRadians(-70)), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(15, -30), Math.toRadians(-90))
                .setVelConstraint(new TranslationalVelocityConstraint(13))
                .splineToConstantHeading(new Vector2d(15, -48), Math.toRadians(-90))
                .resetVelConstraint()

                .setTangent(Math.toRadians(145))
                .splineToSplineHeading(new Pose2d(-25, -25, Math.toRadians(40)), Math.toRadians(145))

                .waitSeconds(1.25)

                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(29, -25, Math.toRadians(-70)), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(35, -30), Math.toRadians(-90))
                .setVelConstraint(new TranslationalVelocityConstraint(13))
                .splineToConstantHeading(new Vector2d(35, -50), Math.toRadians(-90))
                .resetVelConstraint()

                .setTangent(Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(2, -48, Math.toRadians(0)), Math.toRadians(-135))

                .splineToSplineHeading(new Pose2d(-25, -25, Math.toRadians(40)), Math.toRadians(100))

                .build();

        drive.setPoseEstimate(startingPose);

        waitForStart();
        drive.followTrajectorySequence(autonomous);
    }
}