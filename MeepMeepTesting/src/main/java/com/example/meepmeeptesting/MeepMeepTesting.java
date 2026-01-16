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
                        .setTangent(Math.toRadians(-90))
                        .splineToSplineHeading(new Pose2d(55, -37, Math.toRadians(-155)), Math.toRadians(-90))
                        .splineToConstantHeading(new Vector2d(50, -15), Math.toRadians(0))
                        .build());

        RoadRunnerBotEntity leftRed = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(45, 45, Math.toRadians(180), Math.toRadians(180), 13.47)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(startingLeftRed)
//                        .UNSTABLE_addTemporalMarkerOffset(0, ()->{
//                    thread.start();
//                    camera.setPosition(1);
//                    shooter.setVelocity(motorRPM);
//                    servoShoot.setPosition(servoShootClose);
//                })
                .setTangent(Math.toRadians(-45))
                .splineToConstantHeading(new Vector2d(-18,8),Math.toRadians(-45))
//                .UNSTABLE_addTemporalMarkerOffset(0.4, ()->{
//                    firstAprilTag = true;
//                    servoShoot.setPosition(servoShootOpen);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(1.1, ()->{
//                    lift.setPosition(liftUp);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(1.6, ()->{
//                    lift.setPosition(liftDown);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(1.8, ()->{
//                    movePlate(rotatePlate);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(2.9, ()->{
//                    lift.setPosition(liftUp);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(3.3, ()->{
//                    lift.setPosition(liftDown);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(3.5, ()->{
//                    movePlate(rotatePlate);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(4.6, ()->{
//                    lift.setPosition(liftUp);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(5, ()->{
//                    lift.setPosition(liftDown);
//                    shooter.setVelocity(0);
//                    intake.setPower(1);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(5.5, ()->{
//                    servoShoot.setPosition(servoShootClose);
//                })
                .waitSeconds(5)
                        .setTangent(Math.toRadians(45))
                        .splineToSplineHeading(new Pose2d(-10, 26, Math.toRadians(90)), Math.toRadians(90))

                .setVelConstraint(new TranslationalVelocityConstraint(17))
//                .UNSTABLE_addTemporalMarkerOffset(0, ()->{
//                    movePlateGather(rotatePlate);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(0.5, ()->{
//                    movePlateGather(rotatePlate + 3);
//                })
                .splineToConstantHeading(new Vector2d(-10, 51), Math.toRadians(90))
                .resetVelConstraint()
//                .UNSTABLE_addTemporalMarkerOffset(-0.2, ()->{
//                    if(aprilTagID == 21) movePlateID(rotatePlate, 0.2);
//                    if(aprilTagID == 23) movePlateID(rotatePlate*2, 0.2);
//                    shooter.setVelocity(motorRPM);
//                })
                .setTangent(Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(-18, 8, Math.toRadians(125)), Math.toRadians(180))
//                .UNSTABLE_addTemporalMarkerOffset(0.3, ()->{
//                    servoShoot.setPosition(servoShootOpen);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(0.6, ()->{
//                    intake.setPower(0);
//                    lift.setPosition(liftUp);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(1.2, ()->{
//                    lift.setPosition(liftDown);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(1.4, ()->{
//                    movePlate(rotatePlate);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(2.5, ()->{
//                    lift.setPosition(liftUp);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(2.9, ()->{
//                    lift.setPosition(liftDown);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(3.1, ()->{
//                    movePlate(rotatePlate);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(4.2, ()->{
//                    lift.setPosition(liftUp);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(4.6, ()->{
//                    lift.setPosition(liftDown);
//                    shooter.setVelocity(0);
//                    intake.setPower(1);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(5.1, ()->{
//                    servoShoot.setPosition(servoShootClose);
//                })
                .waitSeconds(4.6)
                .setTangent(0)
                .splineToSplineHeading(new Pose2d(5, 18, Math.toRadians(90)), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(12, 18), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(16, 23), Math.toRadians(90))
                .setVelConstraint(new TranslationalVelocityConstraint(17))
//                .UNSTABLE_addTemporalMarkerOffset(0.3, ()->{
//                    movePlateGather(rotatePlate);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(0.8, ()->{
//                    movePlateGather(rotatePlate + 3);
//                    shooter.setVelocity(motorRPM);
//                })
                .splineToConstantHeading(new Vector2d(16, 56), Math.toRadians(90))
                .resetVelConstraint()
//                .UNSTABLE_addTemporalMarkerOffset(-0.2, ()->{
//                    if(aprilTagID == 22) movePlateID((rotatePlate)*2 + 5, 0.2);
//                    if(aprilTagID == 23) movePlateID(rotatePlate + 5, 0.2);
//                })
                .setTangent(Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(-20, 8, Math.toRadians(125)), Math.toRadians(180))
//                .UNSTABLE_addTemporalMarkerOffset(0.7, ()->{
//                    servoShoot.setPosition(servoShootOpen);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(1.1, ()->{
//                    intake.setPower(0);
//                    lift.setPosition(liftUp);
//                    shooter.setVelocity(1075);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(1.6, ()->{
//                    lift.setPosition(liftDown);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(1.8, ()->{
//                    movePlate(rotatePlate);
//                    shooter.setVelocity(motorRPM);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(2.2, ()->{
//                    lift.setPosition(liftUp);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(3.4, ()->{
//                    lift.setPosition(liftDown);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(3.8, ()->{
//                    movePlate(rotatePlate);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(4.9, ()->{
//                    lift.setPosition(liftUp);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(5.3, ()->{
//                    lift.setPosition(liftDown);
//                    shooter.setVelocity(0);
//                })
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