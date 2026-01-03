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
                        .setTangent(Math.toRadians(160))
                        .splineToSplineHeading(new Pose2d(-59, -56, Math.toRadians(45)), Math.toRadians(180))
                        .waitSeconds(5.1)
                        .setTangent(Math.toRadians(45))
                        .splineToSplineHeading(new Pose2d(-55, -10, Math.toRadians(90)), Math.toRadians(180))
                        .splineToConstantHeading(new Vector2d(-56.5, -10), Math.toRadians(180))
                        .resetVelConstraint()
                        .splineToConstantHeading(new Vector2d(-56.5, -58), Math.toRadians(270))
                        .waitSeconds(200)
                        .build());

        RoadRunnerBotEntity SplineDown = new DefaultBotBuilder(meepMeep)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(Test2)
                        .splineToConstantHeading(new Vector2d(0, -40),Math.toRadians(-90))
                        .build());

                        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)

                                .addEntity(SplineUp)
//                                .addEntity(SplineDown)
                .start();
    }
}