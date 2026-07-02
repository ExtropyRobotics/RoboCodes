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
        Pose2d startingCloseBlue = new Pose2d(-50, -48, Math.toRadians(60));
        Pose2d startingFarBlue = new Pose2d(61, -8, 0);

        RoadRunnerBotEntity closeBlue = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(74.6044409417, 30, 5.814271363239445, Math.toRadians(180), 11.2)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(startingCloseBlue)
                        .setTangent(Math.toRadians(45))
                        .splineToSplineHeading(new Pose2d(-25, -25, Math.toRadians(40)), Math.toRadians(45))

                        .waitSeconds(1)

                        .splineToSplineHeading(new Pose2d(-12, -25, Math.toRadians(-70)), Math.toRadians(-90))
                        .setVelConstraint(new TranslationalVelocityConstraint(13))
                        .splineToConstantHeading(new Vector2d(-12, -30), Math.toRadians(-90))
                        .splineToConstantHeading(new Vector2d(-12, -48), Math.toRadians(-90))
                        .resetVelConstraint()


                        .setTangent(Math.toRadians(110))
                        .splineToSplineHeading(new Pose2d(-20, -25, Math.toRadians(40)), Math.toRadians(110))

                        .waitSeconds(1)

                        .setTangent(Math.toRadians(0))
                        .splineToSplineHeading(new Pose2d(8, -25, Math.toRadians(-70)), Math.toRadians(0))
                        .splineToConstantHeading(new Vector2d(15, -30), Math.toRadians(-90))
                        .setVelConstraint(new TranslationalVelocityConstraint(13))
                        .splineToConstantHeading(new Vector2d(15, -48), Math.toRadians(-90))
                        .resetVelConstraint()

                        .setTangent(Math.toRadians(145))
                        .splineToSplineHeading(new Pose2d(-25, -25, Math.toRadians(40)), Math.toRadians(145))

                        .waitSeconds(1)

                        .setTangent(Math.toRadians(0))
                        .splineToSplineHeading(new Pose2d(29, -25, Math.toRadians(-70)), Math.toRadians(0))
                        .splineToConstantHeading(new Vector2d(35, -30), Math.toRadians(-90))
                        .setVelConstraint(new TranslationalVelocityConstraint(13))
                        .splineToConstantHeading(new Vector2d(35, -50), Math.toRadians(-90))
                        .resetVelConstraint()

                        .setTangent(Math.toRadians(90))
                        .splineToSplineHeading(new Pose2d(2, -48, Math.toRadians(0)), Math.toRadians(-135))

                        .splineToSplineHeading(new Pose2d(-25, -25, Math.toRadians(40)), Math.toRadians(100))

                        .build());
        RoadRunnerBotEntity farBlue = new DefaultBotBuilder(meepMeep)
                .setConstraints(74.6044409417, 30, 5.814271363239445, Math.toRadians(180), 11.2)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(startingFarBlue)
                        .setTangent(Math.toRadians(180))
                        .splineToConstantHeading(new Vector2d(54, -8),Math.toRadians(180))
                        .splineToConstantHeading(new Vector2d(50, -8), Math.toRadians(-31))
                        .turn(Math.toRadians(31))
                        .setTangent(Math.toRadians(0))
                        .waitSeconds(3)
                        .splineToSplineHeading(new Pose2d(50 ,-58, Math.toRadians(-60)),Math.toRadians(-90))
                        .setVelConstraint(new TranslationalVelocityConstraint(13))
                        .splineToConstantHeading(new Vector2d(60, -59),Math.toRadians(0))
                        .resetVelConstraint()
                        .splineToSplineHeading(new Pose2d(61, -62, Math.toRadians(-90)), Math.toRadians(-90))
                        .setTangent(Math.toRadians(95))
                        .splineToSplineHeading(new Pose2d(50, -8, Math.toRadians(31)),Math.toRadians(95))
                        .waitSeconds(3)
                        .setTangent(Math.toRadians(180))
                        .splineToSplineHeading(new Pose2d(35, -28, Math.toRadians(-90)), Math.toRadians(-90))
                        .setVelConstraint(new TranslationalVelocityConstraint(13))
                        .splineToConstantHeading(new Vector2d(35, -51), Math.toRadians(-90))
                        .resetVelConstraint()
                        .setTangent(Math.toRadians(65))
                        .splineToSplineHeading(new Pose2d(50, -8, Math.toRadians(31)), Math.toRadians(65))
                        .waitSeconds(3)

                        .waitSeconds(17)
                        .build());


        Image img = null;
        //TODO: download the field from https://www.reddit.com/r/FTC/comments/1nalob0/decode_custom_field_images_meepmeep_compatible/
        //      and replace the path below with the path of your file
        try { img = ImageIO.read(new File("C:\\Users\\cecla\\Documents\\GitHub\\RoboCodes\\MeepMeepTesting\\src\\main\\resources\\background\\season-2025-decode\\field-2025-official.png")); }
        catch(IOException e) {}

        meepMeep.setBackground(img)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
//                .addEntity(closeBlue)
                .addEntity(farBlue)
                .start();
    }
}