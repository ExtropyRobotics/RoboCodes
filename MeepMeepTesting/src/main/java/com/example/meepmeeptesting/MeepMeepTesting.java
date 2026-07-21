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


        Pose2d startingCloseBlue = new Pose2d(-50, -48, Math.toRadians(60));

        RoadRunnerBotEntity closeBlue = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(74.6044409417, 30, 5.814271363239445, Math.toRadians(180), 11.2)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(startingCloseBlue)

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
                        .waitSeconds(1)

                        // == THIRD SET (4) ==

                        // Spline to third set
                        .setTangent(Math.toRadians(0))
                        .splineToSplineHeading(new Pose2d(29, -23, Math.toRadians(-70)), Math.toRadians(0))
                        .splineToConstantHeading(new Vector2d(33, -24), Math.toRadians(-90))
                        .setVelConstraint(new TranslationalVelocityConstraint(13))
                        .splineToConstantHeading(new Vector2d(33, -53), Math.toRadians(-90))
                        .resetVelConstraint()
                        .splineToSplineHeading(new Pose2d(33, -63, Math.toRadians(-90)), Math.toRadians(90))

                        // Spline to open gate
                        .splineToSplineHeading(new Pose2d(2, -48, Math.toRadians(0)), Math.toRadians(-90))

                        // Spline to shooting position
                        .splineToSplineHeading(new Pose2d(-18, -16, Math.toRadians(30)), Math.toRadians(100))

                        .waitSeconds(100)

                        .build());

        Pose2d startingCloseRed = new Pose2d(-50, 48, Math.toRadians(-60));

        RoadRunnerBotEntity closeRed = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(74.6044409417, 30, 5.814271363239445, Math.toRadians(180), 11.2)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(startingCloseRed)

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
                        .splineToConstantHeading(new Vector2d(-13, 54), Math.toRadians(-90))

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
                        .waitSeconds(1)

                        // == THIRD SET (4) ==

                        // Spline to third set
                        .setTangent(Math.toRadians(0))
                        .splineToSplineHeading(new Pose2d(29, 23, Math.toRadians(70)), Math.toRadians(0))
                        .splineToConstantHeading(new Vector2d(33, 24), Math.toRadians(90))
                        .setVelConstraint(new TranslationalVelocityConstraint(13))
                        .splineToConstantHeading(new Vector2d(33, 53), Math.toRadians(90))
                        .resetVelConstraint()
                        .splineToSplineHeading(new Pose2d(33, 63, Math.toRadians(90)), Math.toRadians(-90))

                        // Spline to open gate
                        .splineToSplineHeading(new Pose2d(2, 48, Math.toRadians(0)), Math.toRadians(90))

                        // Spline to shooting position
                        .splineToSplineHeading(new Pose2d(-18, 16, Math.toRadians(-30)), Math.toRadians(-100))

                        .waitSeconds(100)

                        .build());


        Pose2d startingFarBlue = new Pose2d(61, -11, 0);

        RoadRunnerBotEntity farBlue = new DefaultBotBuilder(meepMeep)
                .setConstraints(74.6044409417, 30, 5.814271363239445, Math.toRadians(180), 11.2)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(startingFarBlue)

                        // == PRELOAD (1) ==

                        // Spline to shoot preload
                        .setTangent(Math.toRadians(-155))
                        .splineToSplineHeading(new Pose2d(50, -16, Math.toRadians(17)), Math.toRadians(-155))

                        // Give robot time to shoot
                        .setTangent(Math.toRadians(-70))
                        .waitSeconds(3.4)

                        // == HUMAN PLAYER (2) ==

                        // Spline to human player (takes right-most artefact)
                        .splineToSplineHeading(new Pose2d(55, -35.5, Math.toRadians(-80)), Math.toRadians(-70))
                        .splineToConstantHeading(new Vector2d(58, -57), Math.toRadians(-90))
                        .setVelConstraint(new TranslationalVelocityConstraint(9))
                        .splineToConstantHeading(new Vector2d(58, -60), Math.toRadians(-90))

                        // Third ball is NOT worth it </3

                        // Spline to 2nd ball
                        .setTangent(Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(61, -55), Math.toRadians(0))
                        .splineToConstantHeading(new Vector2d(64, -60), Math.toRadians(-90))
                        .resetVelConstraint()

                        // Spline to shooting position
                        .setTangent(Math.toRadians(105))
                        .splineToConstantHeading(new Vector2d(59, -38), Math.toRadians(100))
                        .splineToSplineHeading(new Pose2d(55, -16, Math.toRadians(16)), Math.toRadians(100))

                        .setTangent(Math.toRadians(-160))
                        .waitSeconds(1.9) // give robot time to shoot

                        // == THIRD SET (3) ==

                        // Spline to set #3
                        .setTangent(Math.toRadians(-160))
                        .splineToSplineHeading(new Pose2d(39, -20, Math.toRadians(-65)), Math.toRadians(-160))
                        .splineToConstantHeading(new Vector2d(34, -22), Math.toRadians(-90))
                        .setVelConstraint(new TranslationalVelocityConstraint(11))
                        .splineToConstantHeading(new Vector2d(34, -53), Math.toRadians(-90))
                        .resetVelConstraint()
                        .splineToSplineHeading(new Pose2d(37, -63, Math.toRadians(-90)), Math.toRadians(65))

                        // Spline to shoot
                        .splineToSplineHeading(new Pose2d(55, -16, Math.toRadians(9)), Math.toRadians(65))

                        .waitSeconds(100)

                        .build());

        Pose2d startingFarRed = new Pose2d(61, 11, 0);

        RoadRunnerBotEntity farRed = new DefaultBotBuilder(meepMeep)
                .setConstraints(74.6044409417, 30, 5.814271363239445, Math.toRadians(180), 11.2)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(startingFarRed)

                        // == PRELOAD (1) ==

                        // Spline to shoot preload
                        .setTangent(Math.toRadians(155))
                        .splineToSplineHeading(new Pose2d(50, 16, Math.toRadians(-17)), Math.toRadians(155))

                        // Give robot time to shoot
                        .setTangent(Math.toRadians(70))
                        .waitSeconds(3.4)

                        // == HUMAN PLAYER (2) ==

                        // Spline to human player (takes right-most artefact)
                        .splineToSplineHeading(new Pose2d(55, 35.5, Math.toRadians(80)), Math.toRadians(70))
                        .splineToConstantHeading(new Vector2d(58, 57), Math.toRadians(90))
                        .setVelConstraint(new TranslationalVelocityConstraint(9))
                        .splineToConstantHeading(new Vector2d(58, 60), Math.toRadians(90))

                        // Third ball is NOT worth it </3

                        // Spline to 2nd ball
                        .setTangent(Math.toRadians(-90))
                        .splineToConstantHeading(new Vector2d(61, 55), Math.toRadians(0))
                        .splineToConstantHeading(new Vector2d(64, 60), Math.toRadians(90))
                        .resetVelConstraint()

                        // Spline to shooting position
                        .setTangent(Math.toRadians(-105))
                        .splineToConstantHeading(new Vector2d(59, 38), Math.toRadians(-100))
                        .splineToSplineHeading(new Pose2d(55, 16, Math.toRadians(16)), Math.toRadians(-100))

                        .setTangent(Math.toRadians(160))
                        .waitSeconds(1.9) // give robot time to shoot

                        // == THIRD SET (3) ==

                        // Spline to set #3
                        .setTangent(Math.toRadians(160))
                        .splineToSplineHeading(new Pose2d(39, 20, Math.toRadians(65)), Math.toRadians(160))
                        .splineToConstantHeading(new Vector2d(34, 22), Math.toRadians(90))
                        .setVelConstraint(new TranslationalVelocityConstraint(11))
                        .splineToConstantHeading(new Vector2d(34, 53), Math.toRadians(90))
                        .resetVelConstraint()
                        .splineToSplineHeading(new Pose2d(37, 63, Math.toRadians(90)), Math.toRadians(-65))

                        // Spline to shoot
                        .splineToSplineHeading(new Pose2d(55, 16, Math.toRadians(-9)), Math.toRadians(-65))

                        .waitSeconds(100)
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
//                .addEntity(closeBlue)
//                .addEntity(closeRed)
                .addEntity(farBlue)
//                .addEntity(farRed)
//                .addEntity(HoldBlue)
//                .addEntity(HoldRed)
                .start();
    }
}