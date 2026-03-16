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
        MeepMeep meepMeep = new MeepMeep(770);
        Pose2d startingLeftRed = new Pose2d(-51, 44, Math.toRadians(155));
        Pose2d startingLeftBlue = new Pose2d(-51, -44, Math.toRadians(-125));
        Pose2d startingRightBlue = new Pose2d(58, -10, Math.toRadians(180));

        RoadRunnerBotEntity rightBlue = new DefaultBotBuilder(meepMeep)
                .setConstraints(70, 50, 5.019103844282359, Math.toRadians(180), 15.175)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(startingRightBlue)
                        .setTangent(Math.toRadians(210))
                        // Launch preload
                        .splineToSplineHeading(new Pose2d(50, -15, Math.toRadians(-155)), Math.toRadians(210))
                        .waitSeconds(2.15)

                        // Spline to gather set 3
                        .splineToSplineHeading(new Pose2d(37, -32.5, Math.toRadians(-90)), Math.toRadians(-90))
                        .setVelConstraint(new TranslationalVelocityConstraint(30))
                        .splineToConstantHeading(new Vector2d(37, -49), Math.toRadians(70))
                        .resetVelConstraint()

                        //Spline to launch set 3
                        .splineToSplineHeading(new Pose2d(43.5, -32, Math.toRadians(-155)), Math.toRadians(70))
                        .splineToConstantHeading(new Vector2d(50, -15), Math.toRadians(70))
                        .setTangent(Math.toRadians(-160))
                        .waitSeconds(2.15)

                        // Spline to gather set 2
                        .splineToSplineHeading(new Pose2d(25, -22, Math.toRadians(-90)), Math.toRadians(-160))
                        .splineToConstantHeading(new Vector2d(11, -32.5), Math.toRadians(-90))
                        .setVelConstraint(new TranslationalVelocityConstraint(30))
                        .splineToConstantHeading(new Vector2d(11, -49), Math.toRadians(50))
                        .resetVelConstraint()

//                        // Spline to launch set 2
                        .splineToSplineHeading(new Pose2d(50, -15, Math.toRadians(-155)), Math.toRadians(50))
                        .setTangent(Math.toRadians(-170))
                        .waitSeconds(2.15)

                        // Spline to get set 3
                        .splineToSplineHeading(new Pose2d(18.3, -22, Math.toRadians(-90)), Math.toRadians(-170))
                        .splineToConstantHeading(new Vector2d(-13, -32.5), Math.toRadians(-90))
                        .setVelConstraint(new TranslationalVelocityConstraint(30))
                        .splineToConstantHeading(new Vector2d(-13, -49), Math.toRadians(30))
                        .resetVelConstraint()
//
//                        // Spline to launch set 3
                        .splineToSplineHeading(new Pose2d(50, -15, Math.toRadians(-155)), Math.toRadians(30))
                        .waitSeconds(2.15)

                        .build());

        RoadRunnerBotEntity leftRed = new DefaultBotBuilder(meepMeep)
//                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(45, 45, Math.toRadians(180), Math.toRadians(180), 13.47)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(startingLeftRed)
//                        .UNSTABLE_addTemporalMarkerOffset(0, ()->{
//                    shooter.setVelocity(motorRPM);
//                    lift.setPosition(0.55);
//                })
                .setTangent(Math.toRadians(-45))
                .splineToConstantHeading(new Vector2d(-18,8),Math.toRadians(-45))
//                .UNSTABLE_addTemporalMarkerOffset(1, ()->{
//                    movePlate(rotatePlate + 237, 1);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(3, ()->{
//                    lift.setPosition(liftUp);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(5, ()->{
//                    lift.setPosition(liftDown);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(6, ()->{
//                    movePlate(plate.getCurrentPosition() + rotatePlate, 1);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(7, ()->{
//                    lift.setPosition(liftUp);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(8, ()->{
//                    lift.setPosition(liftDown);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(9, ()->{
//                    movePlate(plate.getCurrentPosition() + rotatePlate, 1);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(11, ()->{
//                    lift.setPosition(liftUp);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(12, ()->{
//                    lift.setPosition(liftDown);
//                    shooter.setVelocity(0);
//                    intake.setPower(1);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(14, ()->{
//                    movePlate(0, 1);
//                })
                        .build());

        RoadRunnerBotEntity leftBlue = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(70, 50, 5.019103844282359, Math.toRadians(180), 15.175)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(startingLeftBlue)
                        //spline to launch preload
                        .setTangent(Math.toRadians(45))
                        .splineToConstantHeading(new Vector2d(-18,-8),Math.toRadians(45))
                        .setTangent(Math.toRadians(-69))
                        .waitSeconds(2.15)

                        //spline to get set 1
                        .splineToSplineHeading(new Pose2d(-13.85, -19, Math.toRadians(-90)), Math.toRadians(-69))
                        .splineToConstantHeading(new Vector2d(-9.7, -30), Math.toRadians(-90))
                        .setVelConstraint(new TranslationalVelocityConstraint(30))
                        .splineToConstantHeading(new Vector2d(-9.7, -49), Math.toRadians(100))
                        .resetVelConstraint()

                        //spline to launch set 1
                        .splineToSplineHeading(new Pose2d(-18, -8, Math.toRadians(-125)), Math.toRadians(100))
                        .setTangent(Math.toRadians(-35))
                        .waitSeconds(2.15)

                        //spline to get set 2
                        .splineToSplineHeading(new Pose2d(-2.75, -19, Math.toRadians(-90)), Math.toRadians(-35))
                        .splineToConstantHeading(new Vector2d(12.5, -30), Math.toRadians(-90))
                        .setVelConstraint(new TranslationalVelocityConstraint(30))
                        .splineToConstantHeading(new Vector2d(12.5, -48), Math.toRadians(-90))
                        .resetVelConstraint()
                        .splineToConstantHeading(new Vector2d(12.5, -49), Math.toRadians(128))

                        //spline to launch set 2
                        .splineToSplineHeading(new Pose2d(-2.75, -28.5, Math.toRadians(-125)), Math.toRadians(128))
                        .splineToConstantHeading(new Vector2d(-18, -8), Math.toRadians(128))
                        .setTangent(Math.toRadians(-60))
                        .waitSeconds(2.15)

                        //spline to gate 1
                        .splineToSplineHeading(new Pose2d(8, -45, Math.toRadians(-135)), Math.toRadians(-60))
                        .splineToConstantHeading(new Vector2d(8, -60), Math.toRadians(-90))
                        .setTangent(Math.toRadians(60))
                        .waitSeconds(3)

                        //spline to launch gate 1
                        .splineToConstantHeading(new Vector2d(8, -45), Math.toRadians(128))
                        .splineToSplineHeading(new Pose2d(-18, -8, Math.toRadians(-135)), Math.toRadians(128))
                        .setTangent(Math.toRadians(-20))
                        .waitSeconds(2.15)

                        //spline to set 3
                        .splineToSplineHeading(new Pose2d(8.35, -19, Math.toRadians(-90)), Math.toRadians(-20))
                        .splineToConstantHeading(new Vector2d(34.7, -30), Math.toRadians(-90))
                        .setVelConstraint(new TranslationalVelocityConstraint(30))
                        .splineToConstantHeading(new Vector2d(34.7, -49), Math.toRadians(128))
                        .resetVelConstraint()

                        //spline to launch set 3
                        .splineToSplineHeading(new Pose2d(-18, -8, Math.toRadians(-135)), Math.toRadians(128))
                        .waitSeconds(2.15)
                        .build());


        Image img = null;
        try { img = ImageIO.read(new File("C:\\Users\\cecla\\Documents\\GitHub\\RoboCodes\\MeepMeepTesting\\src\\main\\resources\\background\\season-2025-decode\\field-2025-official.png")); }
        catch(IOException e) {}

        meepMeep.setBackground(img)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(leftBlue)
                .start();
    }
}