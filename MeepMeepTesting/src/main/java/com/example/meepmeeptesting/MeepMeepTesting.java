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

                        .splineToConstantHeading(new Vector2d(0, 0), Math.toRadians(0))

                        .build());

        Pose2d startingCloseRed = new Pose2d(-1, 1, Math.toRadians(0));

        RoadRunnerBotEntity closeRed = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(74.6044409417, 30, 5.814271363239445, Math.toRadians(180), 11.2)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(startingCloseRed)

                        .splineToConstantHeading(new Vector2d(0, 0), Math.toRadians(0))

                        .build());


        Pose2d startingFarBlue = new Pose2d(-40, 40, Math.toRadians(90));

        RoadRunnerBotEntity farBlue = new DefaultBotBuilder(meepMeep)
                .setConstraints(74.6044409417, 30, 5.814271363239445, Math.toRadians(180), 11.2)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(startingFarBlue)

                        .splineToConstantHeading(new Vector2d(0, 0), Math.toRadians(0))


                        .build());

        Pose2d startingFarRed = new Pose2d(40, -40, 0);

        RoadRunnerBotEntity farRed = new DefaultBotBuilder(meepMeep)
                .setConstraints(74.6044409417, 30, 5.814271363239445, Math.toRadians(180), 11.2)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(startingFarRed)

                        .splineToConstantHeading(new Vector2d(0, 0), Math.toRadians(0))


                        .build());

        Pose2d startingHoldBlue = new Pose2d(-50, -48, Math.toRadians(60));



        Image img = null;

        //TODO: if not on ceclan's laptop (surprisingly), download the field map from https://www.reddit.com/r/FTC/comments/1weleaj/biobuzz_custom_field_images_meepmeep_compatible/
        // then replace the path below with the path of your .png file

        try { img = ImageIO.read(new File("C:\\Users\\cecla\\Documents\\GitHub\\RoboCodes\\MeepMeepTesting\\src\\main\\resources\\background\\season-2026-biobuzz\\biobuzz_field.png")); }
        catch(IOException e) {}

        meepMeep.setBackground(img)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(closeBlue)
//                .addEntity(closeRed)
//                .addEntity(farBlue)
//                .addEntity(farRed)
                .start();
    }
}