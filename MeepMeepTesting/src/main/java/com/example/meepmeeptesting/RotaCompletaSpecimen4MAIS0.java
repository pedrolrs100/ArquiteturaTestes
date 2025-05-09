package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;


public class RotaCompletaSpecimen4MAIS0 {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(500);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 12)
                .setDimensions(16.9291, 16.9291)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-30, 60, Math.toRadians(-90)))
                        //todo: colocar primeiro specimen
                        .setTangent(Math.toRadians(-45))
                        .splineToConstantHeading(new Vector2d(-10, 35), Math.toRadians(-45))
                        //todo: Go to empurrar sample 1
                        .setTangent(Math.toRadians(180))
                        .splineToConstantHeading(new Vector2d(-32, 35), Math.toRadians(-110))
                        .splineToConstantHeading(new Vector2d(-40, 15), Math.toRadians(180))
                        .splineToConstantHeading(new Vector2d(-47, 15), Math.toRadians(180))
                        //todo: Empurrar sample 1
                        .setTangent(Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(-47, 60), Math.toRadians(90))
                        //todo: Go to empurrar sample 2
                        .setTangent(Math.toRadians(-90))
                        .splineToConstantHeading(new Vector2d(-47, 15), Math.toRadians(-90))
                        .splineToConstantHeading(new Vector2d(-57, 15), Math.toRadians(180))

                        //todo: Empurrar sample 2
                        .setTangent(Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(-57, 60), Math.toRadians(90))
                        //todo: Go to pegar sample 2
                        .setTangent(Math.toRadians(-90))
                        .splineToConstantHeading(new Vector2d(-57, 50), Math.toRadians(-90))
                        .splineToConstantHeading(new Vector2d(-47, 50), Math.toRadians(90))
                        //todo:
                        .setTangent(Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(-47, 60), Math.toRadians(90))
                        /*
                        //todo:
                        .setTangent(Math.toRadians(-45))
                        .splineToConstantHeading(new Vector2d(-7, 35), Math.toRadians(-45))
                        //todo:
                        .setTangent(Math.toRadians(135))
                        .splineToConstantHeading(new Vector2d(-47, 60), Math.toRadians(135))
                        //todo:
                        .setTangent(Math.toRadians(-45))
                        .splineToConstantHeading(new Vector2d(-5, 35), Math.toRadians(-45))
                        //todo:
                        .setTangent(Math.toRadians(135))
                        .splineToConstantHeading(new Vector2d(-47, 60), Math.toRadians(135))
                        //todo:
                        .setTangent(Math.toRadians(-45))
                        .splineToConstantHeading(new Vector2d(0, 35), Math.toRadians(-45))
                        //todo:
                        .setTangent(135)
                        .splineToSplineHeading(new Pose2d(-35, 55, Math.toRadians(135)), Math.toRadians(135))
                        */



                        .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}