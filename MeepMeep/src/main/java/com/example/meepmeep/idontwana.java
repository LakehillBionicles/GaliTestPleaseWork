package com.example.meepmeep;
import static java.lang.Thread.sleep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class idontwana {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        Pose2d startPose = new Pose2d(14, 61, Math.toRadians(90));
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(42.5, 62.890756998077265, 3.358942, Math.toRadians(180), 13.4)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPose)
                                .back(10)
                                .turn(Math.toRadians(180))
                                //Switch x and y
                                //make y opposite sign
                                //heading switches sign
                                //Tangent -90
                                .splineToLinearHeading(startPose.minus(new Pose2d(-9, 20, Math.toRadians(180))), Math.toRadians(-90))
                                .waitSeconds(1)
                                .splineToLinearHeading(startPose.minus(new Pose2d(-9, 10, Math.toRadians(180))), Math.toRadians(90))
                                .turn(Math.toRadians(-90))
                                .splineToLinearHeading(startPose.minus(new Pose2d(-35, 3, Math.toRadians(-90))), Math.toRadians(0))

                                //blue far middle
                                //.splineToLinearHeading((startPose).minus(new Pose2d(10, 37, Math.toRadians(90))),Math.toRadians(0))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
