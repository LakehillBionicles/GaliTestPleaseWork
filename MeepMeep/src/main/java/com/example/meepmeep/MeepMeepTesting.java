package com.example.meepmeep;
import static java.lang.Thread.sleep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
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
                                .splineToLinearHeading((startPose).plus(new Pose2d(-8.3, -17, Math.toRadians(180))),Math.toRadians(-90))
                                .waitSeconds(1)
                                .back(5)
                                .turn(Math.toRadians(-90))
                                .splineToLinearHeading((startPose).plus(new Pose2d(3, -2, Math.toRadians(90))),Math.toRadians(0))
                                .splineToLinearHeading((startPose).plus(new Pose2d(70, -2, Math.toRadians(90))),Math.toRadians(0))
                                .splineToLinearHeading((startPose).plus(new Pose2d(80, -30, Math.toRadians(90))),Math.toRadians(-90))
                                .splineToLinearHeading((startPose).plus(new Pose2d(90, -32, Math.toRadians(90))),Math.toRadians(0))

                           //blue far middle
                                /*
                                .back(5)
                                .turn(Math.toRadians(-90))
                                .splineToLinearHeading((startPose).minus(new Pose2d(10, 37, Math.toRadians(90))),Math.toRadians(0))
                                .splineToLinearHeading((startPse).minus(new Pose2d(-4, 37, Math.toRadians(90))),Math.toRadians(0))
                                .splineToLinearHeading((startPose).minus(new Pose2d(3, 37, Math.toRadians(90))),Math.toRadians(180))
                                .waitSeconds(1)
                                .back(10)
                                .turn(Math.toRadians(180))
                                .splineToLinearHeading((startPose).minus(new Pose2d(18, 50, Math.toRadians(-90))),Math.toRadians(180))
                                .waitSeconds(1)
                                .back(20)
                                .splineToLinearHeading((startPose).minus(new Pose2d(-65, 50, Math.toRadians(-90))),Math.toRadians(0))
                                .splineToLinearHeading((startPose).minus(new Pose2d(-85, 30, Math.toRadians(-90))),Math.toRadians(0))
                                .waitSeconds(2)
                                .splineToLinearHeading((startPose).minus(new Pose2d(-88, 30, Math.toRadians(-90))),Math.toRadians(0))
                                .waitSeconds(1)
                                .splineToLinearHeading((startPose).minus(new Pose2d(-85, 30, Math.toRadians(-90))),Math.toRadians(180))
                                .splineToLinearHeading((startPose).minus(new Pose2d(-65, 50, Math.toRadians(-90))),Math.toRadians(180))
                                .splineToLinearHeading((startPose).minus(new Pose2d(18, 50, Math.toRadians(-90))),Math.toRadians(180))
                                ///blue far right(from robot)

                                 */
                                //.splineToLinearHeading((startPose).minus(new Pose2d(12, 30, Math.toRadians(-90))),Math.toRadians(0))

                                /*
                                .back(5)
                                .splineToLinearHeading((startPose).minus(new Pose2d(8, 15, Math.toRadians(180))),Math.toRadians(-90))
                                .splineToLinearHeading((startPose).minus(new Pose2d(-4, 13, Math.toRadians(180))),Math.toRadians(-90))
                                .splineToLinearHeading((startPose).minus(new Pose2d(-4, 30, Math.toRadians(180))),Math.toRadians(-90))
                                .splineToLinearHeading((startPose).minus(new Pose2d(16, 50, Math.toRadians(-90))),Math.toRadians(180))
                                .waitSeconds(1)
                                .back(20)
                                .splineToLinearHeading((startPose).minus(new Pose2d(-65, 50, Math.toRadians(-90))),Math.toRadians(0))
                                .splineToLinearHeading((startPose).minus(new Pose2d(-85, 45, Math.toRadians(-90))),Math.toRadians(0))

                                 *///Left far blue
                                /*
                                .back(5)
                                .turn(Math.toRadians(-90))
                                .splineToLinearHeading((startPose).minus(new Pose2d(4, 25, Math.toRadians(90))),Math.toRadians(-90))
                                .splineToLinearHeading((startPose).minus(new Pose2d(-10, 25, Math.toRadians(90))),Math.toRadians(0))
                                .splineToLinearHeading((startPose).minus(new Pose2d(-3, 27, Math.toRadians(90))),Math.toRadians(180))
                                .waitSeconds(1)
                                .back(5)
                                .turn(Math.toRadians(180))
                                .splineToLinearHeading((startPose).minus(new Pose2d(16, 50, Math.toRadians(-90))),Math.toRadians(180))
                                .waitSeconds(1)
                                .back(20)
                                .splineToLinearHeading((startPose).minus(new Pose2d(-65, 50, Math.toRadians(-90))),Math.toRadians(0))
                                .splineToLinearHeading((startPose).minus(new Pose2d(-85, 40, Math.toRadians(-90))),Math.toRadians(0))
                                */
                                .build()
                );
        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}