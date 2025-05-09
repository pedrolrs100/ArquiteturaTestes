package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;


public class RotaCompletaSample0MAIS4 {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 12)
                .setDimensions(16.9291, 16.9291)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(35, 62, Math.toRadians(180)))
                        //todo: DEPOSIT
                        .setTangent(Math.toRadians(-45))
                        .splineToLinearHeading(new Pose2d(51, 54, Math.toRadians(-135)), Math.toRadians(45))
                        //todo: go to sample two
                        .setTangent(Math.toRadians(-90))
                        .splineToLinearHeading(new Pose2d(47.5, 50, Math.toRadians(-90)), Math.toRadians(-90))
                        //todo: DEPOSIT
                        .setTangent(Math.toRadians(90))
                        .splineToLinearHeading(new Pose2d(51, 54, Math.toRadians(-135)), Math.toRadians(45))
                        //todo: go to sample three
                        .setTangent(Math.toRadians(-45))
                        .splineToLinearHeading(new Pose2d(58, 50, Math.toRadians(-90)), Math.toRadians(-45))
                        //todo: DEPOSIT
                        .setTangent(Math.toRadians(180))
                        .splineToLinearHeading(new Pose2d(51, 54, Math.toRadians(-135)), Math.toRadians(135))
                        //todo:go to sample four
                        .setTangent(Math.toRadians(180))
                        .splineToLinearHeading(new Pose2d(46, 48, Math.toRadians(-45)), Math.toRadians(180))
                        //todo: DEPOSIT
                        .setTangent(Math.toRadians(-45))
                        .splineToLinearHeading(new Pose2d(51, 54, Math.toRadians(-135)), Math.toRadians(45))
                        //todo: GO TO HOUSE
                        .setTangent(Math.toRadians(-135))
                        .splineToLinearHeading(new Pose2d(30, 10, Math.toRadians(180)), Math.toRadians(-135))








                        .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}