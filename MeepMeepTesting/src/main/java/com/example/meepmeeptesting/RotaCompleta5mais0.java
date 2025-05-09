package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;


public class RotaCompleta5mais0 {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(500);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 12)
                .setDimensions(16.9291, 16.9291)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(10, -60, Math.toRadians(90)))
                        //todo read to collect
                        .setTangent(Math.toRadians(90))
                        .splineToLinearHeading(new Pose2d(35,-40,Math.toRadians(60)),Math.toRadians(90))

                        //todo collect
                        .splineToSplineHeading(new Pose2d(35, -40 - 4, Math.toRadians(60 + 2)), Math.toRadians(90))//todo -10
                        .splineToSplineHeading(new Pose2d(35, -40 - 2, Math.toRadians(60 - 2)), Math.toRadians(90))//todo -10
                        .splineToSplineHeading(new Pose2d(35, -40, Math.toRadians(60)), Math.toRadians(90))//todo -10

                        //todo cuspir
                        .splineToLinearHeading(new Pose2d(35,-45,Math.toRadians(-60)),Math.toRadians(-90))

                        //todo read to collect 2
                        .setTangent(Math.toRadians(90))
                        .splineToLinearHeading(new Pose2d(45,-40,Math.toRadians(60)),Math.toRadians(90))

                        //todo collect 2
                        .splineToSplineHeading(new Pose2d(45, -40 - 4, Math.toRadians(60 + 2)), Math.toRadians(90))//todo -10
                        .splineToSplineHeading(new Pose2d(45, -40 - 2, Math.toRadians(60 - 2)), Math.toRadians(90))//todo -10
                        .splineToSplineHeading(new Pose2d(45, -40, Math.toRadians(60)), Math.toRadians(90))//todo -10

                        //todo cuspir 2
                        .splineToLinearHeading(new Pose2d(45,-45,Math.toRadians(-60)),Math.toRadians(-90))

                        //todo read to collect 3
                        .setTangent(Math.toRadians(90))
                        .splineToLinearHeading(new Pose2d(50,-35,Math.toRadians(50)),Math.toRadians(90))

                        //todo collect 3
                        .splineToSplineHeading(new Pose2d(50, -35 - 4, Math.toRadians(60 + 2)), Math.toRadians(90))//todo -10
                        .splineToSplineHeading(new Pose2d(50, -35 - 2, Math.toRadians(60 - 2)), Math.toRadians(90))//todo -10
                        .splineToSplineHeading(new Pose2d(50, -35, Math.toRadians(60)), Math.toRadians(90))//todo -10

                        //todo cuspir 3
                        .setTangent(Math.toRadians(180))
                        .splineToLinearHeading(new Pose2d(40,-45,Math.toRadians(-70)),Math.toRadians(-45))




                        .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}