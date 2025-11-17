package org.firstinspires.ftc.teamcode.auto;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ArmControler;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


@Autonomous (name = "RegioLeft")

public class RegioLeft extends LinearOpMode {

    public int targetAx = 0;
    public int targetSlider = 0;
    public double clawPoz = 0.06;
    public double wristPlace = 0;
    public double maxVel = 25;
    public double power = 0.4;

    Pose2d startingPoseRegioLeft = new Pose2d(-36, -60,Math.toRadians(90));

    public class ArmThreadLeft extends Thread{
        ArmControler brat;
        public ArmThreadLeft(ArmControler brat){
            this.brat = brat;

            brat.setPowerSlider(1);
            brat.setPower(power);
            brat.setAxPoz(targetAx);
            brat.setSliderPoz(targetSlider);
            brat.setClaw(clawPoz);
            brat.setWrist(wristPlace);
        }
        @Override
        public void run(){
            while(opModeInInit()){
                telemetry.addLine("in init");
                telemetry.update();
            }
            while(opModeIsActive()){
                brat.setAxPoz(targetAx);
                brat.setSliderPoz(targetSlider);
                brat.setClaw(clawPoz);
                brat.setWrist(wristPlace);

                brat.callTelemetry();
                telemetry.update();
            }
        }
    }

    @Override
    public void runOpMode(){
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(),telemetry);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        ArmThreadLeft thread = new ArmThreadLeft(new ArmControler(hardwareMap,telemetry));

        TrajectorySequence RegioLeft= drive.trajectorySequenceBuilder(startingPoseRegioLeft)
                .UNSTABLE_addTemporalMarkerOffset(0.5, ()-> {
                    targetAx = 445;
                    targetSlider = 1600;
                    wristPlace = 0;
                })
                .splineToConstantHeading(new Vector2d(-10, -36), Math.toRadians(90))
                .build();

        drive.setPoseEstimate(startingPoseRegioLeft);
        thread.start();
        waitForStart();
        sleep(500);
        drive.followTrajectorySequence(RegioLeft);

        }
}