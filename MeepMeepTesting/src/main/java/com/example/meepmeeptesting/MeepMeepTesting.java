package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        Pose2d Test = new Pose2d(0, 0, Math.toRadians(90));
        Pose2d Test2 = new Pose2d(0, 0, Math.toRadians(-90));

        RoadRunnerBotEntity SplineUp = new DefaultBotBuilder(meepMeep)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(Test)
                        .splineToConstantHeading(new Vector2d(0, 40),Math.toRadians(90))
                        .build());

        RoadRunnerBotEntity SplineDown = new DefaultBotBuilder(meepMeep)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(Test2)
                        .splineToConstantHeading(new Vector2d(0, -40),Math.toRadians(-90))
                        .build());

                        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)

                                .addEntity(SplineUp)
                                .addEntity(SplineDown)
                .start();
    }
}