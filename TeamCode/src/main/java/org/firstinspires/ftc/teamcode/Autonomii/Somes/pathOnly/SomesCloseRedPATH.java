package org.firstinspires.ftc.teamcode.Autonomii.Somes.pathOnly;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous (name = "CloseRedPATH", group = "Path")

public class SomesCloseRedPATH extends LinearOpMode {
    Pose2d startingPose = new Pose2d(0, 0, Math.toRadians(180));

    @Override
    public void runOpMode(){
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        TrajectorySequence exampleAuto = drive.trajectorySequenceBuilder(startingPose)


                .build();

        drive.setPoseEstimate(startingPose);

        waitForStart();
        drive.followTrajectorySequence(exampleAuto);
    }
}