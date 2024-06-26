package com.example.meepmeep.NewMEEPMEES;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class leftRedFare {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        Pose2d startPose = new Pose2d(-38, -61, Math.toRadians(-90));
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(42.5, 62.890756998077265, 3.358942, Math.toRadians(180), 13.4)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPose)
                                .back(5)
                                .splineToLinearHeading((startPose).plus(new Pose2d(-8, 19, Math.toRadians(0))),Math.toRadians(90))

                                //drop pixels
                                .forward(5)
                                .turn(Math.toRadians(-90))
                                .waitSeconds(0.2)
                                .splineToLinearHeading((startPose).plus(new Pose2d(-20, 13, Math.toRadians(-90))), Math.toRadians(180))
                                .waitSeconds(0.2)
                                .splineToLinearHeading((startPose).plus(new Pose2d(-20, 30.5, Math.toRadians(-90))), Math.toRadians(90))
                                .waitSeconds(1)
                                //grab pixel
                                .back(1)
                                .strafeRight(5)
                                .splineToLinearHeading((startPose).plus(new Pose2d(-18, 50, Math.toRadians(-90))), Math.toRadians(90))
                                .back(10)
                                .splineToLinearHeading((startPose).plus(new Pose2d(0, 51.5, Math.toRadians(-90))),Math.toRadians(0))
                                .splineToLinearHeading((startPose).plus(new Pose2d(68, 52.5, Math.toRadians(-90))),Math.toRadians(0))
                                .splineToLinearHeading((startPose).plus(new Pose2d(75, 26, Math.toRadians(-90))),Math.toRadians(-90))
                                .waitSeconds(1)
                                .build()
                );
        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }

}