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


@Autonomous (name = "!NatioLeftSAMPLE")

public class NatioLeftSample extends LinearOpMode {

    public int targetAx = 0;
    public int targetSlider = 0;
    public double clawClose = 0.18;
    public double clawOpen = 0;
    public double clawPoz = clawClose;
    public double wristBack = 0;
    public double wristParallel = 0.4;
    public double wristWall = 0.2;
    public double wristStraight = 0.6;
    public double wristPoz = wristBack;
    public double maxVel = 19;
    public double power = 0.4;

    Pose2d startingPoseNatioLeft = new Pose2d(-32.7, -60,Math.toRadians(90));

    public class ArmThreadLeft extends Thread{
        ArmControler brat;
        public ArmThreadLeft(ArmControler brat){
            this.brat = brat;

            brat.setPowerSlider(1);
            brat.setPower(power);
            brat.setAxPoz(targetAx);
            brat.setSliderPoz(targetSlider);
            brat.setClaw(clawPoz);
            brat.setWrist(wristPoz);
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
                brat.setWrist(wristPoz);

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

        TrajectorySequence NatioLeft= drive.trajectorySequenceBuilder(startingPoseNatioLeft)
                .setVelConstraint(new TranslationalVelocityConstraint(maxVel))
                .UNSTABLE_addTemporalMarkerOffset(0, ()->{
                    targetAx = 850;
                    wristPoz = wristParallel;
                })
                .setTangent(Math.toRadians(160))
                .splineToSplineHeading(new Pose2d(-59, -56, Math.toRadians(45)), Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(0.4, ()->{
                    targetSlider = 2099;
                })
                .UNSTABLE_addTemporalMarkerOffset(2.8, ()->{
                    targetAx = 890;
                })
                .UNSTABLE_addTemporalMarkerOffset(3, ()->{
                    wristPoz = wristBack;
                })
                .UNSTABLE_addTemporalMarkerOffset(3.5, ()->{
                    clawPoz = clawOpen;
                })
                .UNSTABLE_addTemporalMarkerOffset(3.8, ()->{
                    wristPoz = wristParallel;
                    targetAx = 850;
                })
                .UNSTABLE_addTemporalMarkerOffset(3.9, ()->{
                    targetSlider = 511;
                })
                .waitSeconds(4)
                .setTangent(Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(-51, -37.2, Math.toRadians(90)), Math.toRadians(90))
                .UNSTABLE_addTemporalMarkerOffset(-0.6, ()->{
                    targetAx = 100;
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.3, ()->{
                    targetSlider = 511;
                    targetAx = 81;
                    wristPoz = wristParallel;
                })
                .UNSTABLE_addTemporalMarkerOffset(0.3, ()->{
                    clawPoz = clawClose;
                })
                .UNSTABLE_addTemporalMarkerOffset(0.4, ()->{
                    targetSlider = 1;
                })
                .UNSTABLE_addTemporalMarkerOffset(0.8, ()->{
                    targetAx = 890;
                    wristPoz = wristStraight;
                })
                .waitSeconds(0.5)
                .setTangent(Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(-59, -56, Math.toRadians(45)), Math.toRadians(-135))
                .UNSTABLE_addTemporalMarkerOffset(0.3, ()->{
                    targetSlider = 2099;
                })
                .UNSTABLE_addTemporalMarkerOffset(2.9, ()->{
                    wristPoz = wristBack;
                })
                .UNSTABLE_addTemporalMarkerOffset(3.4, ()->{
                    clawPoz = clawOpen;
                })
                .UNSTABLE_addTemporalMarkerOffset(3.7, ()->{
                    wristPoz = wristParallel;
                    targetAx = 850;
                })
                .UNSTABLE_addTemporalMarkerOffset(3.8, ()->{
                    targetSlider = 511;
                })
                .waitSeconds(3.9)
                .setTangent(Math.toRadians(75))
                .splineToSplineHeading(new Pose2d(-61, -38.2, Math.toRadians(90)), Math.toRadians(90))
                .UNSTABLE_addTemporalMarkerOffset(-0.4, ()->{
                    targetAx = 100;
                })
                .UNSTABLE_addTemporalMarkerOffset(0.7, ()->{
                    targetAx = 81;
                })
                .UNSTABLE_addTemporalMarkerOffset(1, ()->{
                    clawPoz = clawClose;
                })
                .UNSTABLE_addTemporalMarkerOffset(1.2, ()->{
                    targetSlider = 1;
                })
                .UNSTABLE_addTemporalMarkerOffset(1.3, ()->{
                    targetAx = 890;
                    wristPoz = wristStraight;
                })
                .waitSeconds(1.2)
                .setTangent(Math.toRadians(-45))
                .splineToSplineHeading(new Pose2d(-59, -56, Math.toRadians(45)), Math.toRadians(-135))
                .UNSTABLE_addTemporalMarkerOffset(0.3, ()->{
                    targetSlider = 2099;
                })
                .UNSTABLE_addTemporalMarkerOffset(2.9, ()->{
                    wristPoz = wristBack;
                })
                .UNSTABLE_addTemporalMarkerOffset(3.4, ()->{
                    clawPoz = clawOpen;
                })
                .UNSTABLE_addTemporalMarkerOffset(3.7, ()->{
                    wristPoz = wristParallel;
                    targetAx = 850;
                })
                .UNSTABLE_addTemporalMarkerOffset(3.8, ()->{
                    targetSlider = 0;
                })
                .UNSTABLE_addTemporalMarkerOffset(5.5, ()->{
                    targetAx = 0;
                    wristPoz = wristBack;
                })
                .waitSeconds(5.1)
                .setTangent(Math.toRadians(45))
                .splineToSplineHeading(new Pose2d(-55, -10, Math.toRadians(90)), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-56.5, -10), Math.toRadians(180))
                .resetVelConstraint()
                .splineToConstantHeading(new Vector2d(-56.5, -58), Math.toRadians(270))
                .waitSeconds(200)
                .build();

        drive.setPoseEstimate(startingPoseNatioLeft);
        thread.start();
        waitForStart();
        sleep(500);
        drive.followTrajectorySequence(NatioLeft);

    }
}