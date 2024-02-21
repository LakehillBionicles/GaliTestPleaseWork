package com.example.meepmeep;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class redFareRightAutoMeepMeep {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        Pose2d startPose = new Pose2d(-38, -61, Math.toRadians(-90));
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(42.5, 62.890756998077265, 3.358942, Math.toRadians(180), 13.4)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPose)
                                .back(30)
                                .turn(Math.toRadians(90))
                                .splineToLinearHeading((startPose).plus(new Pose2d(3, 34, Math.toRadians(90))),Math.toRadians(0))
                                //drop pixels
                                .waitSeconds(1)
                                .back(6)
                                .splineToLinearHeading((startPose).plus(new Pose2d(-16, 49.5, Math.toRadians(-90))),Math.toRadians(180))
                                .forward(4)
                                //grab pixel
                                .waitSeconds(1)
                                .back(10)
                                .splineToLinearHeading((startPose).plus(new Pose2d(0, 51.5, Math.toRadians(-90))),Math.toRadians(0))
                                .splineToLinearHeading((startPose).plus(new Pose2d(65, 51.5, Math.toRadians(-90))),Math.toRadians(0))
                                .splineToLinearHeading((startPose).plus(new Pose2d(80, 34, Math.toRadians(-90))),Math.toRadians(-90))
                                .splineToLinearHeading((startPose).plus(new Pose2d(87, 20, Math.toRadians(-90))),Math.toRadians(0))
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
