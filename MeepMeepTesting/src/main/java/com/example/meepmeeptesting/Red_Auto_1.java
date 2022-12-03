package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class Red_Auto_1 {
    static int auto = 0;

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);


        if (auto == 0) {
            //RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
            // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
            RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                    .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 10)
                    .followTrajectorySequence(drive ->
                            drive.trajectorySequenceBuilder(new Pose2d(-34.5, -60.5, Math.toRadians(90)))

                                    /*.lineTo(new Vector2d(-60.5, -60.5))
                                   // .lineTo(new Vector2d(-10, -10))
                                    .lineToLinearHeading(new Pose2d(-60.5, -35, Math.toRadians(90)))
                                    .build()*/
                                    .splineTo(new Vector2d(-60.5, -48),Math.toRadians(90))
                                    .splineTo(new Vector2d(-10, -10 ), Math.toRadians(90))
                                    .build()
                    );
            meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                    .setDarkMode(true)
                    .setBackgroundAlpha(0.95f)
                    .addEntity(myBot)
                    .start();
        }
        if (auto == 1) {
            //RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
            // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
            RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                    .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 10)
                    .followTrajectorySequence(drive ->
                            drive.trajectorySequenceBuilder(new Pose2d(-34.5, -60.5, Math.toRadians(90)))

                                   // .lineTo(new Vector2d(-34.5, -60.5))
                                    .lineToLinearHeading(new Pose2d(-34.5, -10, Math.toRadians(90)))
                                    .build()
                    );
            meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                    .setDarkMode(true)
                    .setBackgroundAlpha(0.95f)
                    .addEntity(myBot)
                    .start();
        }
        if (auto == 2) {
            //RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
            // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
            RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                    .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 10)
                    .followTrajectorySequence(drive ->
                            drive.trajectorySequenceBuilder(new Pose2d(-34.5, -60.5, Math.toRadians(90)))

                                    .lineTo(new Vector2d(-10, -60.5))
                                    .lineTo(new Vector2d(-10, -10))
                                    .lineToLinearHeading(new Pose2d(10, -10, Math.toRadians(135)))
                                    .build()
                    );
            meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                    .setDarkMode(true)
                    .setBackgroundAlpha(0.95f)
                    .addEntity(myBot)
                    .start();
        }
    }
}