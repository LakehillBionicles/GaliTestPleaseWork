package com.example.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class blueFareRightAutoMeepMeep {

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        Pose2d startPose = new Pose2d(-38, 61, Math.toRadians(90));
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(42.5, 62.890756998077265, 3.358942, Math.toRadians(180), 13.4)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPose)
                                .back(5)
                                .turn(Math.toRadians(180))
                                .splineToLinearHeading((startPose).plus(new Pose2d(-4, -17, Math.toRadians(180))),Math.toRadians(-90))
                                //drop pixels
                                .back(5)
                                .turn(Math.toRadians(-90))
                                .waitSeconds(0.2)
                                .strafeRight(5)
                                .splineToLinearHeading((startPose).plus(new Pose2d(-20, -15, Math.toRadians(95))), Math.toRadians(180))
                                .waitSeconds(0.2)
                                .strafeLeft(17)
                                .waitSeconds(0.1)
                                .strafeRight(2)
                                .waitSeconds(0.1)
                                .forward(1)
                                .waitSeconds(1)
                                //grab pixel
                                .turn(Math.toRadians(5))
                                .back(2)
                                .strafeLeft(20)
                                .splineToLinearHeading((startPose).plus(new Pose2d(-13, -51.5, Math.toRadians(100))), Math.toRadians(0))
                                .splineToLinearHeading((startPose).plus(new Pose2d(65, -49.5, Math.toRadians(100))), Math.toRadians(0))
                                .splineToLinearHeading((startPose).plus(new Pose2d(78, -50, Math.toRadians(100))), Math.toRadians(0))
                                .splineToLinearHeading((startPose).plus(new Pose2d(87, -32, Math.toRadians(100))), Math.toRadians(0))
                                .build()
                );
        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
