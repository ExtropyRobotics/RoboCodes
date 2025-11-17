package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);
        Pose2d labubu67 = new Pose2d(55, -30, Math.toRadians(105));

        RoadRunnerBotEntity RegioTestLeft = new DefaultBotBuilder(meepMeep)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(labubu67)
//                        .splineToConstantHeading(new Vector2d(0, 0), Math.toRadians(45))
//                        .splineToSplineHeading(new Pose2d(50, 40, Math.toRadians(0)),Math.toRadians(0))
                        .splineToSplineHeading(new Pose2d(30, 40, Math.toRadians(105)),Math.toRadians(105))
                        .build());

                        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                                .addEntity(RegioTestLeft)
//                                .addEntity(RegioTestRight)
                .start();
    }
}