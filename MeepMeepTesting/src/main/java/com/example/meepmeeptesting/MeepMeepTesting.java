package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Image;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);
        Pose2d startingLeftRed = new Pose2d(-51, 44, Math.toRadians(125));
        Pose2d startingLeftBlue = new Pose2d(-51, -44, Math.toRadians(-125));
        Pose2d startingRightBlue = new Pose2d(58, -10, Math.toRadians(180));

        RoadRunnerBotEntity rightBlue = new DefaultBotBuilder(meepMeep)
                .setConstraints(45, 45, Math.toRadians(180), Math.toRadians(180), 13.47)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(startingRightBlue)
                        .setTangent(Math.toRadians(180))
                        .splineToConstantHeading(new Vector2d(54, -10), Math.toRadians(180))
                        .splineToSplineHeading(new Pose2d(50, -15, Math.toRadians(-155)), Math.toRadians(-90))
                        .waitSeconds(5.6)
                        .setTangent(180)
                        .splineToSplineHeading(new Pose2d(36, -25, Math.toRadians(-90)), Math.toRadians(-90))
                        .setVelConstraint(new TranslationalVelocityConstraint(18))
                        .splineToConstantHeading(new Vector2d(36, -60), Math.toRadians(-90))
                        .resetVelConstraint()
                        .setTangent(Math.toRadians(90))
                        .splineToSplineHeading(new Pose2d(50, -15, Math.toRadians(-155)), Math.toRadians(0))
                        .waitSeconds(5.9)
                        .setTangent(Math.toRadians(-90))
                        .splineToSplineHeading(new Pose2d(53, -55, Math.toRadians(-30)), Math.toRadians(-90))
                        .setVelConstraint(new TranslationalVelocityConstraint(18))
                        .splineToConstantHeading(new Vector2d(60, -57), Math.toRadians(180))
                        .setTangent(Math.toRadians(-90))
                        .splineToSplineHeading(new Pose2d(60, -60, Math.toRadians(-45)), Math.toRadians(-90))
                        .resetVelConstraint()
                        .setTangent(Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(55, -37), Math.toRadians(90))
                        .setTangent(Math.toRadians(90))
                        .splineToSplineHeading(new Pose2d(50, -15, Math.toRadians(-155)), Math.toRadians(90))
                        .build());

        RoadRunnerBotEntity leftRed = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(45, 45, Math.toRadians(180), Math.toRadians(180), 13.47)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(startingLeftRed)
//                        .UNSTABLE_addTemporalMarkerOffset(0, ()->{
//                            shooter.setVelocity(1000);
//                            if(aprilTagID == 22) movePlate(179*2);
//                            if(aprilTagID == 23) movePlate(179);
//                        })
                        .setTangent(Math.toRadians(-45))
                        .splineToConstantHeading(new Vector2d(-31,21),Math.toRadians(-45))
//                        .UNSTABLE_addTemporalMarkerOffset(1, ()->{
//                            lift.setPosition(0.8);
//                            shooter.setVelocity(1100);
//                        })
//                        .UNSTABLE_addTemporalMarkerOffset(1.5, ()->{
//                            lift.setPosition(0.46);
//                        })
//                        .UNSTABLE_addTemporalMarkerOffset(1.7, ()->{
//                            movePlate(179);
//                        })
//                        .UNSTABLE_addTemporalMarkerOffset(2.7, ()->{
//                            lift.setPosition(0.8);
////                    shooter.setVelocity(1100);
//                        })
//                        .UNSTABLE_addTemporalMarkerOffset(3.2, ()->{
//                            lift.setPosition(0.46);
//                        })
//                        .UNSTABLE_addTemporalMarkerOffset(3.4, ()->{
//                            movePlate(179);
//                        })
//                        .UNSTABLE_addTemporalMarkerOffset(4.4, ()->{
//                            lift.setPosition(0.8);
//                        })
//                        .UNSTABLE_addTemporalMarkerOffset(4.9, ()->{
//                            lift.setPosition(0.46);
//                            shooter.setVelocity(0);
//                            intake.setPower(1);
//                        })
                        .waitSeconds(4.9)
                        .setTangent(Math.toRadians(-45))
                        .splineToSplineHeading(new Pose2d(-13, 18, Math.toRadians(90)), Math.toRadians(0))
                        .splineToConstantHeading(new Vector2d(-8, 26), Math.toRadians(90))
                        .setVelConstraint(new TranslationalVelocityConstraint(10))
//                        .UNSTABLE_addTemporalMarkerOffset(0.4, ()->{
//                            movePlate(179);
//                        })
//                        .UNSTABLE_addTemporalMarkerOffset(1.2, ()->{
//                            movePlate(179);
//                        })
//                        .UNSTABLE_addTemporalMarkerOffset(1.6, ()->{
//                            shooter.setVelocity(1000);
//                        })
//                        .UNSTABLE_addTemporalMarkerOffset(2.1, ()->{
//                            intake.setPower(0);
//                            if(aprilTagID == 21) movePlate(179);
//                            if(aprilTagID == 23) movePlate(179*2);
//                        })
                        .splineToConstantHeading(new Vector2d(-8, 45), Math.toRadians(90))
                        .resetVelConstraint()
                        .setTangent(Math.toRadians(-90))
                        .splineToSplineHeading(new Pose2d(-30, 20, Math.toRadians(120)), Math.toRadians(180))
//                        .UNSTABLE_addTemporalMarkerOffset(0.6, ()->{
//                            lift.setPosition(0.8);
//                            shooter.setVelocity(1000);
//                        })
//                        .UNSTABLE_addTemporalMarkerOffset(1.1, ()->{
//                            lift.setPosition(0.46);
//                        })
//                        .UNSTABLE_addTemporalMarkerOffset(1.3, ()->{
//                            movePlate(179);
//                        })
//                        .UNSTABLE_addTemporalMarkerOffset(2.2, ()->{
//                            lift.setPosition(0.8);
////                    shooter.setVelocity(1100);
//                        })
//                        .UNSTABLE_addTemporalMarkerOffset(2.8, ()->{
//                            lift.setPosition(0.46);
//                        })
//                        .UNSTABLE_addTemporalMarkerOffset(3, ()->{
//                            movePlate(179);
//                        })
//                        .UNSTABLE_addTemporalMarkerOffset(4, ()->{
//                            lift.setPosition(0.8);
//                        })
//                        .UNSTABLE_addTemporalMarkerOffset(4.5, ()->{
//                            lift.setPosition(0.46);
//                            shooter.setVelocity(0);
//                            intake.setPower(1);
//                        })
                        .waitSeconds(4.5)
                        .setTangent(Math.toRadians(0))
                        .splineToSplineHeading(new Pose2d(5, 18, Math.toRadians(90)), Math.toRadians(0))
                        .splineToConstantHeading(new Vector2d(13, 26), Math.toRadians(90))
                        .setVelConstraint(new TranslationalVelocityConstraint(10))
//                        .UNSTABLE_addTemporalMarkerOffset(0.7, ()->{
//                            movePlate(179);
//                        })
//                        .UNSTABLE_addTemporalMarkerOffset(1.5, ()->{
//                            movePlate(179);
//                        })
//                        .UNSTABLE_addTemporalMarkerOffset(2.1, ()->{
//                            shooter.setVelocity(1000);
//                        })
//                        .UNSTABLE_addTemporalMarkerOffset(2.2, ()->{
//                            intake.setPower(0);
//                            if(aprilTagID == 22) movePlate(179*2);
//                            if(aprilTagID == 23) movePlate(179);
//                        })
                        .splineToConstantHeading(new Vector2d(13, 45), Math.toRadians(-90))
                        .resetVelConstraint()
                        .setTangent(Math.toRadians(-90))
                        .splineToSplineHeading(new Pose2d(-30, 20, Math.toRadians(120)), Math.toRadians(180))
//                        .UNSTABLE_addTemporalMarkerOffset(1, ()->{
//                            lift.setPosition(0.8);
//                            shooter.setVelocity(1000);
//                        })
//                        .UNSTABLE_addTemporalMarkerOffset(1.5, ()->{
//                            lift.setPosition(0.46);
//                        })
//                        .UNSTABLE_addTemporalMarkerOffset(1.7, ()->{
//                            movePlate(179);
//                        })
//                        .UNSTABLE_addTemporalMarkerOffset(2.6, ()->{
//                            lift.setPosition(0.8);
////                    shooter.setVelocity(1100);
//                        })
//                        .UNSTABLE_addTemporalMarkerOffset(3.2, ()->{
//                            lift.setPosition(0.46);
//                        })
//                        .UNSTABLE_addTemporalMarkerOffset(3.4, ()->{
//                            movePlate(179);
//                        })
//                        .UNSTABLE_addTemporalMarkerOffset(4.4, ()->{
//                            lift.setPosition(0.8);
//                        })
//                        .UNSTABLE_addTemporalMarkerOffset(4.8, ()->{
//                            lift.setPosition(0.46);
//                            shooter.setVelocity(0);
//                            intake.setPower(0);
//                        })
                        .waitSeconds(200)
                        .build());

        RoadRunnerBotEntity leftBlue = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(45, 45, Math.toRadians(180), Math.toRadians(180), 13.47)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(startingLeftBlue)
                        .setTangent(0)
                        .splineToSplineHeading(new Pose2d(5, -18, Math.toRadians(-90)), Math.toRadians(0))
                        .splineToConstantHeading(new Vector2d(12, -18), Math.toRadians(0))
                        .splineToConstantHeading(new Vector2d(18, -23), Math.toRadians(-90))
                        .setVelConstraint(new TranslationalVelocityConstraint(17))
                        .splineToConstantHeading(new Vector2d(18, -59), Math.toRadians(-90))
                        .resetVelConstraint()
                .waitSeconds(200)
                        .build());


        Image img = null;
        try { img = ImageIO.read(new File("C:\\Users\\cecla\\Documents\\GitHub\\RoboCodes\\MeepMeepTesting\\src\\main\\resources\\background\\season-2025-decode\\field-2025-official.png")); }
        catch(IOException e) {}

        meepMeep.setBackground(img)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(leftRed)
                .addEntity(leftBlue)
                .addEntity(rightBlue)
                .start();
    }
}