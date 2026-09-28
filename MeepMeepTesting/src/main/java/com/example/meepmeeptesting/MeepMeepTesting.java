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


        Pose2d startingPose = new Pose2d(-30, 30, Math.toRadians(90));

        RoadRunnerBotEntity closeBlue = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(74.6044409417, 30, 5.814271363239445, Math.toRadians(180), 11.2)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(startingPose)

                        .setTangent(Math.toRadians(-45))
                        .splineToSplineHeading(new Pose2d(30, -30, Math.toRadians(-90)), Math.toRadians(-45))
                        .setTangent(Math.toRadians(-90))
                        .setVelConstraint(new TranslationalVelocityConstraint(10))
                        .splineToSplineHeading(new Pose2d(30, -60, Math.toRadians(-90)), Math.toRadians(-90))

                        .resetVelConstraint()

                        .build());

        Pose2d startingCloseRed = new Pose2d(-1, 1, Math.toRadians(0));

        RoadRunnerBotEntity closeRed = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(74.6044409417, 30, 5.814271363239445, Math.toRadians(180), 11.2)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(startingCloseRed)

                        .setTangent(Math.toRadians(0))
                        .splineToSplineHeading(new Pose2d(1, 1, Math.toRadians(-90)), Math.toRadians(0))
                        .setTangent(Math.toRadians(-90))
                        .splineToSplineHeading(new Pose2d(1, -1, Math.toRadians(180)), Math.toRadians(-90))
                        .setTangent(Math.toRadians(180))
                        .splineToSplineHeading(new Pose2d(-1, -1, Math.toRadians(90)), Math.toRadians(180))
                        .setTangent(Math.toRadians(90))
                        .splineToSplineHeading(new Pose2d(-1, 1, Math.toRadians(0)), Math.toRadians(90))


                        .build());


        Pose2d startingFarBlue = new Pose2d(-40, 40, Math.toRadians(90));

        RoadRunnerBotEntity farBlue = new DefaultBotBuilder(meepMeep)
                .setConstraints(74.6044409417, 30, 5.814271363239445, Math.toRadians(180), 11.2)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(startingFarBlue)

                        .setTangent(Math.toRadians(90))
                        .splineToSplineHeading(new Pose2d(0, 50, Math.toRadians(-90)), Math.toRadians(90))
                        .setTangent(Math.toRadians(-90))
                        .splineToSplineHeading(new Pose2d(50, 40, Math.toRadians(90)), Math.toRadians(-90))


                        .build());

        Pose2d startingFarRed = new Pose2d(40, -40, 0);

        RoadRunnerBotEntity farRed = new DefaultBotBuilder(meepMeep)
                .setConstraints(74.6044409417, 30, 5.814271363239445, Math.toRadians(180), 11.2)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(startingFarRed)

                        .setTangent(Math.toRadians(-90))
                        .splineToSplineHeading(new Pose2d(0, -50, Math.toRadians(-90)), Math.toRadians(-90))
                        .setTangent(Math.toRadians(90))
                        .splineToSplineHeading(new Pose2d(-50, -40, Math.toRadians(90)), Math.toRadians(90))


                        .build());

        Pose2d startingHoldBlue = new Pose2d(-50, -48, Math.toRadians(60));

        RoadRunnerBotEntity HoldBlue = new DefaultBotBuilder(meepMeep)
                .setConstraints(74.6044409417, 30, 5.814271363239445, Math.toRadians(180), 11.2)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(startingHoldBlue)

                        // == PRELOAD (1) ==

                        // Spline to first shooting position
                        .setTangent(Math.toRadians(45))
                        .splineToSplineHeading(new Pose2d(-18, -18, Math.toRadians(50)), Math.toRadians(45))

                        // Give robot time to shoot
                        .waitSeconds(0.9)

                        // == FIRST SET (2) ==

                        // Spline to the first set
                        .splineToSplineHeading(new Pose2d(-13, -17, Math.toRadians(-70)), Math.toRadians(-90))
                        .setVelConstraint(new TranslationalVelocityConstraint(13))
                        .splineToConstantHeading(new Vector2d(-10, -30), Math.toRadians(-90))
                        .splineToConstantHeading(new Vector2d(-10, -45), Math.toRadians(-90))
                        .resetVelConstraint()
                        .splineToConstantHeading(new Vector2d(-13, -54), Math.toRadians(90))

                        // Spline to shooting position.
                        .splineToSplineHeading(new Pose2d(-20, -18, Math.toRadians(45)), Math.toRadians(110))

                        // Give robot time to shoot
                        .waitSeconds(0.9)

                        // == SECOND SET (3) ==

                        // Spline to second set
                        .setTangent(Math.toRadians(0))
                        .splineToSplineHeading(new Pose2d(4, -15, Math.toRadians(-65)), Math.toRadians(0))
                        .splineToConstantHeading(new Vector2d(9, -15), Math.toRadians(0))
                        .splineToConstantHeading(new Vector2d(11, -16), Math.toRadians(-90))
                        .setVelConstraint(new TranslationalVelocityConstraint(13))
                        .splineToConstantHeading(new Vector2d(15, -53), Math.toRadians(-90))
                        .resetVelConstraint()
                        .splineToSplineHeading(new Pose2d(15, -62, Math.toRadians(-90)), Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(15, -50), Math.toRadians(90))

                        // Spline to shooting position
                        .splineToSplineHeading(new Pose2d(-20, -18, Math.toRadians(35)), Math.toRadians(125))

                        // Give robot time to shoot
                        .setTangent(Math.toRadians(0))
                        .waitSeconds(1)

                        // Spline to open gate
                        .splineToSplineHeading(new Pose2d(2, -48, Math.toRadians(0)), Math.toRadians(-90))

                        .waitSeconds(1)

                        .build());

        Pose2d startingHoldRed = new Pose2d(-50, 48, Math.toRadians(-60));

        RoadRunnerBotEntity HoldRed = new DefaultBotBuilder(meepMeep)
                .setConstraints(74.6044409417, 30, 5.814271363239445, Math.toRadians(180), 11.2)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(startingHoldRed)

                        // == PRELOAD (1) ==

                        // Spline to first shooting position
                        .setTangent(Math.toRadians(-45))
                        .splineToSplineHeading(new Pose2d(-18, 18, Math.toRadians(-50)), Math.toRadians(-45))

                        // Give robot time to shoot
                        .waitSeconds(0.9)

                        // == FIRST SET (2) ==

                        // Spline to the first set
                        .splineToSplineHeading(new Pose2d(-13, 17, Math.toRadians(70)), Math.toRadians(90))
                        .setVelConstraint(new TranslationalVelocityConstraint(13))
                        .splineToConstantHeading(new Vector2d(-10, 30), Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(-10, 45), Math.toRadians(90))
                        .resetVelConstraint()
                        .splineToConstantHeading(new Vector2d(-13, 54), Math.toRadians(90))

                        // Spline to open gate
                        .setTangent(Math.toRadians(-90))
                        .splineToSplineHeading(new Pose2d(-7, 46, Math.toRadians(180)), Math.toRadians(0))
                        .splineToConstantHeading(new Vector2d(2, 48), Math.toRadians(90))
                        .waitSeconds(1)

                        // Spline to shooting position.
                        .splineToSplineHeading(new Pose2d(-20, 18, Math.toRadians(-45)), Math.toRadians(-110))

                        // Give robot time to shoot
                        .waitSeconds(0.9)

                        // == SECOND SET (3) ==

                        // Spline to second set
                        .setTangent(Math.toRadians(0))
                        .splineToSplineHeading(new Pose2d(4, 15, Math.toRadians(65)), Math.toRadians(0))
                        .splineToConstantHeading(new Vector2d(9, 15), Math.toRadians(0))
                        .splineToConstantHeading(new Vector2d(11, 16), Math.toRadians(90))
                        .setVelConstraint(new TranslationalVelocityConstraint(13))
                        .splineToConstantHeading(new Vector2d(15, 53), Math.toRadians(90))
                        .resetVelConstraint()
                        .splineToSplineHeading(new Pose2d(15, 62, Math.toRadians(90)), Math.toRadians(-90))
                        .splineToConstantHeading(new Vector2d(15, 50), Math.toRadians(-90))

                        // Spline to shooting position
                        .splineToSplineHeading(new Pose2d(-20, 18, Math.toRadians(-35)), Math.toRadians(-125))

                        // Give robot time to shoot
                        .setTangent(Math.toRadians(0))
                        .waitSeconds(1)

                        .splineToSplineHeading(new Pose2d(-7, 48, Math.toRadians(0)), Math.toRadians(90))

                        .waitSeconds(1)

                        .build());



        Image img = null;

        //TODO: if not on ceclan's laptop (surprisingly), download the field map from https://www.reddit.com/r/FTC/comments/1nalob0/decode_custom_field_images_meepmeep_compatible/
        // then replace the path below with the path of your .png file

        try { img = ImageIO.read(new File("C:\\Users\\cecla\\Documents\\GitHub\\RoboCodes\\MeepMeepTesting\\src\\main\\resources\\background\\season-2025-decode\\field-2025-official.png")); }
        catch(IOException e) {}

        meepMeep.setBackground(img)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(closeBlue)
//                .addEntity(closeRed)
//                .addEntity(farBlue)
//                .addEntity(farRed)
//                .addEntity(HoldBlue)
//                .addEntity(HoldRed)
                .start();
    }
}