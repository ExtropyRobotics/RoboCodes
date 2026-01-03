package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Image;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(45, 45, Math.toRadians(180), Math.toRadians(180), 13.47)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(90)))
                        .setTangent(Math.toRadians(45))
                        .splineToConstantHeading(new Vector2d(20,10),Math.toRadians(135))
                        .splineToConstantHeading(new Vector2d(-20,20),Math.toRadians(150))
                        .splineToConstantHeading(new Vector2d(0,50),Math.toRadians(45))
                        .splineToConstantHeading(new Vector2d(25,-55),Math.toRadians(-150))

                        .build());


        Image img = null;
        try { img = ImageIO.read(new File("C:\\Users\\cecla\\Documents\\GitHub\\RoboCodes\\MeepMeepTesting\\src\\main\\resources\\background\\season-2025-decode\\field-2025-official.png")); }
        catch(IOException e) {}

        meepMeep.setBackground(img)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}