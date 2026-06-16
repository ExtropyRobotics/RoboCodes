package org.firstinspires.ftc.teamcode.Autonomii.Somes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Disabled
@Autonomous (name = "FarBlue")

public class SomesFarBlue extends LinearOpMode {
    Pose2d startingPose = new Pose2d(0, 0, Math.toRadians(180));
    // 0 - right
    // 90 - up
    // 180 - left
    // -90 (270) - down

    class autoThread implements Runnable {
        @Override
        public void run() {

            while (opModeIsActive() && !isStopRequested()) {

                // Thread content here

            }
        }
    }

    autoThread obj = new autoThread();

    @Override
    public void runOpMode(){
        Thread thread = new Thread(obj);
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(),telemetry);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // HardwareMap content here

        TrajectorySequence exampleAuto = drive.trajectorySequenceBuilder(startingPose)

                // Splines and unstables here

                //ex:

                //.setVelConstraint(new TranslationalVelocityConstraint(0))
                //.setTangent(Math.toRadians(0))
                //.splineToSplineHeading(new Pose2d(0, 0, Math.toRadians(-90)), Math.toRadians(0))
                //.splineToConstantHeading(new Vector2d(0, 0), Math.toRadians(-90))

                //.UNSTABLE_addTemporalMarkerOffset(0.5, ()->{

                // Unstable contents here

                //})

                .build();

        drive.setPoseEstimate(startingPose);

        waitForStart();
        thread.start();
        drive.followTrajectorySequence(exampleAuto);
    }
}