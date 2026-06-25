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
        Pose2d starting = new Pose2d(0, 0, Math.toRadians(90));

        RoadRunnerBotEntity leftBlue = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(74.6044409417, 30, 5.814271363239445, Math.toRadians(180), 11.2)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(starting)

                        .setTangent(Math.toRadians(-45))
                        .splineToSplineHeading(new Pose2d(40, -40, Math.toRadians(0)), Math.toRadians(-45))
                        .setTangent(Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(40, 40), Math.toRadians(90))
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