package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);


        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 40, Math.toRadians(142), Math.toRadians(142), 15.66)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-32, -62, Math.toRadians(90)))
                                .forward(0.01)
                                /*.lineToSplineHeading(new Pose2d(15,-34.3, Math.toRadians(180)))
                                .back(10)
                                .lineToSplineHeading(new Pose2d(52 , -29, Math.toRadians(0)))
                                .lineToSplineHeading(new Pose2d(18,-8, Math.toRadians(180)))
                                .lineToConstantHeading(new Vector2d(-30, -8))
                                .lineToConstantHeading(new Vector2d(-55, -11.5))
                                .lineToLinearHeading(new Pose2d(-35 ,-5, Math.toRadians(0)))
                                .splineToConstantHeading(new Vector2d(10, -5), Math.toRadians(0))
                                .splineToConstantHeading(new Vector2d(49, -37), Math.toRadians(0))
                                .forward(5)
                                .back(5)
                                .strafeLeft(25)
                                .forward(5)*/
                                .build()
                );



        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}