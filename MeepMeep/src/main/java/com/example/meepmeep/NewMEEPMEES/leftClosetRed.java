package com.example.meepmeep.NewMEEPMEES;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class leftClosetRed {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        Pose2d startPose = new Pose2d(15, -61, Math.toRadians(-90));
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(42.5, 62.890756998077265, 3.358942, Math.toRadians(180), 13.4)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPose)
                                .back(1)
                                //.waitSeconds(0.1)
                                //.strafeLeft(10)
                                //.waitSeconds(0.1)
                                //.back(2)
                                .splineTo((new Vector2d(startPose.getX(),startPose.getY())).plus(new Vector2d(10, 14)), Math.toRadians(90))
                                .splineTo((new Vector2d(startPose.getX(),startPose.getY())).plus(new Vector2d(3, 26)), Math.toRadians(-180))
                                .splineTo((new Vector2d(startPose.getX(),startPose.getY())).plus(new Vector2d(-6, 26)), Math.toRadians(-180))
                                .forward(11)
                                .turn(Math.toRadians(180))
                                .back(2)
                                .splineToLinearHeading((startPose).plus(new Pose2d(32, 32, Math.toRadians(-90))),Math.toRadians(0))
                                //drop pixels
                                .build()
                );
        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}