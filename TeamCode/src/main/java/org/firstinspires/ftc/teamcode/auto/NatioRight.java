package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ArmControler;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


@Autonomous (name = "!NatioRight")
public class NatioRight extends LinearOpMode {
    public Pose2d startingPoseNatioRight = new Pose2d(15, -60, Math.toRadians(90));
    public int targetAx = 0;
    public int targetSlider = 0;
    public double clawClose = 0.15;
    public double clawChamber = 0.112;
    public double clawOpen = 0;
    public double clawPoz = clawClose;
    public double wristBack = 0;
    public double wristParallel = 0.4;
    public double wristWall = 0.117;
    public double wristStraight = 0.6;
    public double wristPoz = wristBack;
    public double maxVel = 24;
    public double power = 0.4;

    public class ArmThreadRight extends Thread{
        ArmControler brat;
        public ArmThreadRight(ArmControler brat){
            this.brat = brat;

            brat.setPower(0.3);
            brat.setPowerSlider(1);
            brat.setAxPoz(targetAx);
            brat.setSliderPoz(targetSlider);
            brat.setClaw(clawClose);
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
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        ArmThreadRight thread = new ArmThreadRight(new ArmControler(hardwareMap,telemetry));

        TrajectorySequence natioRight = drive.trajectorySequenceBuilder(startingPoseNatioRight)
                .UNSTABLE_addTemporalMarkerOffset(0, ()->{
                    targetAx = 430;
                    targetSlider = 1500;
                    wristPoz = wristBack;
                })
                .waitSeconds(0.5)
                .setVelConstraint(new TranslationalVelocityConstraint(16))
                .setTangent(Math.toRadians(170))
                .splineToConstantHeading(new Vector2d(5, -33), Math.toRadians(90))
                .waitSeconds(0.05)
                .UNSTABLE_addTemporalMarkerOffset(-0.2, ()->{
                    clawPoz = clawOpen;
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.1, ()->{
                    targetSlider = 0;
                })
                .waitSeconds(0.2)
                .UNSTABLE_addTemporalMarkerOffset(0.3, ()->{
                    targetAx = 117;
                    wristPoz = wristWall;
                })
                .setVelConstraint(new TranslationalVelocityConstraint(maxVel))
                .setTangent(Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(20, -40, Math.toRadians(-90)), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(30, -40), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(35, -25), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(40, -5), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(45, -5), Math.toRadians(-90))
                .resetVelConstraint()
                .splineToConstantHeading(new Vector2d(45, -47), Math.toRadians(-90))
                .setVelConstraint(new TranslationalVelocityConstraint(maxVel))
                .splineToConstantHeading(new Vector2d(45, -15), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(55, -5), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(55, -6), Math.toRadians(-90))
                .resetVelConstraint()
                .splineToConstantHeading(new Vector2d(50, -40), Math.toRadians(-90))
                .setVelConstraint(new TranslationalVelocityConstraint(18))
                .splineToConstantHeading(new Vector2d(50, -57.2), Math.toRadians(-90))
                .resetVelConstraint()
                .UNSTABLE_addTemporalMarkerOffset(0.3, ()->{
                    clawPoz = clawClose;
                })
                .UNSTABLE_addTemporalMarkerOffset(0.6, ()->{
                    wristPoz = wristBack;
                })
                .UNSTABLE_addDisplacementMarkerOffset(0.5, ()->{
                    targetAx = 900;
                })
                .UNSTABLE_addTemporalMarkerOffset(0.7, ()->{
                    clawPoz = clawChamber;
                })
                .waitSeconds(0.7)
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(0, -28), Math.toRadians(90))
                .UNSTABLE_addTemporalMarkerOffset(0, ()->{
                    targetSlider = 550;
                })
                .UNSTABLE_addTemporalMarkerOffset(0.4, ()->{
                    clawPoz = clawOpen;
                })
                .UNSTABLE_addTemporalMarkerOffset(0.6, ()->{
                    targetSlider = 0;
                })
                .UNSTABLE_addTemporalMarkerOffset(1, ()->{
                    targetAx = 117;
                    wristPoz = wristWall;
                })
                .waitSeconds(0.9)
                .setTangent(Math.toRadians(-135))
                .splineToConstantHeading(new Vector2d(50, -40), Math.toRadians(0))
                .setVelConstraint(new TranslationalVelocityConstraint(18))
                .setTangent(Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(50, -57), Math.toRadians(-90))
                .resetVelConstraint()
                .UNSTABLE_addTemporalMarkerOffset(0.2, ()->{
                    clawPoz = clawClose;
                })
                .UNSTABLE_addTemporalMarkerOffset(0.5, ()->{
                    wristPoz = wristBack;
                })
                .UNSTABLE_addTemporalMarkerOffset(0.4, ()->{
                    targetAx = 900;
                })
                .UNSTABLE_addTemporalMarkerOffset(0.8, ()->{
                    clawPoz = clawChamber;
                })
                .waitSeconds(0.7)
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(2, -28), Math.toRadians(90))
                .UNSTABLE_addTemporalMarkerOffset(0.4, ()->{
                    targetSlider = 550;
                })
                .UNSTABLE_addTemporalMarkerOffset(0.8, ()->{
                    clawPoz = clawOpen;
                })
                .UNSTABLE_addTemporalMarkerOffset(1, ()->{
                    targetSlider = 0;
                })
                .UNSTABLE_addTemporalMarkerOffset(1.2, ()->{
                    targetAx = 0;
                    wristPoz = wristBack;
                })
                .waitSeconds(0.5)
                .setTangent(Math.toRadians(-135))
                .splineToConstantHeading(new Vector2d(55, -55), Math.toRadians(-90))
                .build();
        drive.setPoseEstimate(startingPoseNatioRight);
        thread.start();
        telemetry.addLine("inside initialization");
        telemetry.addData("state ", thread.getState());
        telemetry.update();

        waitForStart();

        sleep(500);
        drive.followTrajectorySequence(natioRight);
    }
}