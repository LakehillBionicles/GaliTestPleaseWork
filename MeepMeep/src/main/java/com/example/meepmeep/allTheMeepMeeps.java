package com.example.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.Constraints;
import com.noahbres.meepmeep.roadrunner.DriveTrainType;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class allTheMeepMeeps {

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        Pose2d startPose = new Pose2d(15, 61, Math.toRadians(90));
        ColorSchemeBlueDark colorSchemeBlueDark = new ColorSchemeBlueDark();
        ColorSchemeRedDark colorSchemeRedDark = new ColorSchemeRedDark();
        Constraints myBotConstraints = new Constraints(42.5, 62.890756998077265, 3.358942, Math.toRadians(180), 13.4);
        RoadRunnerBotEntity closeRobotBlue = new RoadRunnerBotEntity(meepMeep,
                myBotConstraints,
                16,
                18,
                startPose,
                colorSchemeBlueDark,
                1,
                DriveTrainType.MECANUM,
                true);
        Pose2d startPoseBlueFar = new Pose2d(-34, 61, Math.toRadians(90));
        RoadRunnerBotEntity farRobotBlue = new RoadRunnerBotEntity(meepMeep,
                myBotConstraints,
                16,
                18,
                startPose,
                colorSchemeBlueDark,
                1,
                DriveTrainType.MECANUM,
                true);
        Pose2d startPoseRedFar = new Pose2d(-34, -61, Math.toRadians(-90));
        RoadRunnerBotEntity farRobotRed = new RoadRunnerBotEntity(meepMeep,
                myBotConstraints,
                16,
                18,
                startPose,
                colorSchemeRedDark,
                1,
                DriveTrainType.MECANUM,
                true);
        Pose2d startPoseRedClose = new Pose2d(15, -61, Math.toRadians(-90));
        RoadRunnerBotEntity closeRobotRed = new RoadRunnerBotEntity(meepMeep,
                myBotConstraints,
                16,
                18,
                startPose,
                colorSchemeRedDark,
                1,
                DriveTrainType.MECANUM,
                true);
        Pose2d startPoseRedWallrider = new Pose2d(-34, -61, Math.toRadians(-90));
        RoadRunnerBotEntity wallriderRobotRed = new RoadRunnerBotEntity(meepMeep,
                myBotConstraints,
                16,
                18,
                startPose,
                colorSchemeRedDark,
                1,
                DriveTrainType.MECANUM,
                true);
        Pose2d startPoseBlueWallrider = new Pose2d(-34, 61, Math.toRadians(90));
        RoadRunnerBotEntity wallriderRobotBlue = new RoadRunnerBotEntity(meepMeep,
                myBotConstraints,
                16,
                18,
                startPose,
                colorSchemeBlueDark,
                1,
                DriveTrainType.MECANUM,
                true);
        // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                /*
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPose)
                                .back(15)
                                .turn(Math.toRadians(90))
                                .back(4)
                                .splineToLinearHeading((startPose).plus(new Pose2d(-11, 20, Math.toRadians(90))), Math.toRadians(180))
                                .splineToLinearHeading((startPose).plus(new Pose2d(-11, 41, Math.toRadians(90))), Math.toRadians(90))
                                //drop pixels
                                .waitSeconds(1)
                                .back(3)
                                .splineToLinearHeading((startPose).plus(new Pose2d(-16, 49.5, Math.toRadians(-90))), Math.toRadians(180))
                                .forward(4)
                                //grab pixel
                                .waitSeconds(1)
                                .back(10)
                                .splineToLinearHeading((startPose).plus(new Pose2d(0, 51.5, Math.toRadians(-90))), Math.toRadians(0))
                                .splineToLinearHeading((startPose).plus(new Pose2d(65, 51.5, Math.toRadians(-90))), Math.toRadians(0))
                                .splineToLinearHeading((startPose).plus(new Pose2d(80, 34, Math.toRadians(-90))), Math.toRadians(-90))
                                .splineToLinearHeading((startPose).plus(new Pose2d(87, 26, Math.toRadians(-90))), Math.toRadians(0))
                                .waitSeconds(1)
                                .build()
                );

                 */

        closeRobotBlue.followTrajectorySequence(closeRobotBlue.getDrive()
                .trajectorySequenceBuilder(startPose)
                .back(4)
                .splineToLinearHeading((startPose).plus(new Pose2d(12, -40, Math.toRadians(90))), Math.toRadians(-90))
                .waitSeconds(1)
                //drop pixels
                .back(1)
                .splineToLinearHeading((startPose).plus(new Pose2d(33, -26, Math.toRadians(90))), Math.toRadians(0))
                .waitSeconds(1)
                //grab pixels
                .splineToLinearHeading((startPose).plus(new Pose2d(30, -1, Math.toRadians(90))), Math.toRadians(0))
                .back(13)
                .build()
        );
        farRobotBlue.followTrajectorySequence(closeRobotBlue.getDrive()
                .trajectorySequenceBuilder(startPoseBlueFar)
                .back(27)
                .turn(Math.toRadians(-90))
                .forward(10)
                .back(5)
                .waitSeconds(1)
                //drop pixels
                .back(5)
                .turn(Math.toRadians(180))
                .waitSeconds(0.2)
                .splineToLinearHeading((startPoseBlueFar).plus(new Pose2d(-19, -15, Math.toRadians(95))), Math.toRadians(180))
                .waitSeconds(0.2)
                .strafeLeft(17)
                .waitSeconds(0.1)
                .strafeRight(3)
                .waitSeconds(0.1)
                .forward(3)
                .waitSeconds(1)
                //grab pixels
                .turn(Math.toRadians(-5))
                .back(3)
                .splineToLinearHeading((startPoseBlueFar).plus(new Pose2d(-17, -51.5, Math.toRadians(90))), Math.toRadians(0))
                .splineToLinearHeading((startPoseBlueFar).plus(new Pose2d(65, -49.5, Math.toRadians(90))), Math.toRadians(0))
                .splineToLinearHeading((startPoseBlueFar).plus(new Pose2d(78, -50, Math.toRadians(90))), Math.toRadians(90))
                .splineToLinearHeading((startPoseBlueFar).plus(new Pose2d(86, -22, Math.toRadians(90))), Math.toRadians(0))
                .build()
        );
        closeRobotRed.followTrajectorySequence(closeRobotRed.getDrive()
                .trajectorySequenceBuilder(startPoseRedClose)
                .back(4)
                        .turn(Math.toRadians(180))
                .splineToLinearHeading((startPoseRedClose).plus(new Pose2d(2, 15, Math.toRadians(180))), Math.toRadians(90))
                .waitSeconds(1)
                //drop pixels
                .back(2)
                .waitSeconds(0.1)
                .strafeRight(7)
                .splineToLinearHeading((startPoseRedClose).plus(new Pose2d(32, 24, Math.toRadians(-85))), Math.toRadians(0))
                .waitSeconds(1)
                //grab pixels
                .splineToLinearHeading((startPoseRedClose).plus(new Pose2d(30, 1, Math.toRadians(-85))), Math.toRadians(0))
                .back(9)
                .build()
        );
        farRobotRed.followTrajectorySequence(farRobotRed.getDrive()
                .trajectorySequenceBuilder(startPoseRedFar)
                .back(30)
                .turn(Math.toRadians(90))
                .splineToLinearHeading((startPoseRedFar).plus(new Pose2d(5, 33, Math.toRadians(90))),Math.toRadians(0))
                .waitSeconds(1)
                //drop pixels
                .back(5)
                .turn(Math.toRadians(180))
                .waitSeconds(0.2)
                .splineToLinearHeading((startPoseRedFar).plus(new Pose2d(-20, 13, Math.toRadians(-90))), Math.toRadians(180))
                .waitSeconds(0.2)
                .splineToLinearHeading((startPoseRedFar).plus(new Pose2d(-20, 30.5, Math.toRadians(-90))), Math.toRadians(90))
                .waitSeconds(0.2)
                .splineToLinearHeading((startPoseRedFar).plus(new Pose2d(-20, 27, Math.toRadians(-90))), Math.toRadians(-90))
                .waitSeconds(1)
                //grab pixels
                .back(4)
                .splineToLinearHeading((startPoseRedFar).plus(new Pose2d(-15, 53.5, Math.toRadians(-90))),Math.toRadians(0))
                .splineToLinearHeading((startPoseRedFar).plus(new Pose2d(75, 53.5, Math.toRadians(-90))),Math.toRadians(0))
                .splineToLinearHeading((startPoseRedFar).plus(new Pose2d(80, 34, Math.toRadians(-90))),Math.toRadians(-90))
                .splineToLinearHeading((startPoseRedFar).plus(new Pose2d(87, 28, Math.toRadians(-90))),Math.toRadians(0))
                .build()
        );
        wallriderRobotRed.followTrajectorySequence(wallriderRobotRed.getDrive()
                        .trajectorySequenceBuilder(startPoseRedWallrider)
                .back(27)
                .turn(Math.toRadians(90))
                .forward(10)
                .back(10)
                .splineToLinearHeading((startPoseRedWallrider).plus(new Pose2d(6, 33, Math.toRadians(90))),Math.toRadians(0))
                .waitSeconds(1)
                .back(5)
                .turn(Math.toRadians(180))
                .waitSeconds(0.2)
                .splineToLinearHeading((startPoseRedWallrider).plus(new Pose2d(-24, 13, Math.toRadians(-90))), Math.toRadians(180))
                .waitSeconds(0.2)
                .strafeRight(1)
                .splineToLinearHeading((startPoseRedWallrider).plus(new Pose2d(-23, 30.5, Math.toRadians(-90))), Math.toRadians(90))
                .waitSeconds(1)
                .back(1)
                .splineToLinearHeading((startPoseRedWallrider).plus(new Pose2d(-18, 10, Math.toRadians(-90))), Math.toRadians(-90))
                .waitSeconds(1)
                .turn(Math.toRadians(3))
                .back(1)
                .splineToLinearHeading((startPoseRedWallrider).plus(new Pose2d(-7, 5, Math.toRadians(-87))), Math.toRadians(0))
                .splineToLinearHeading((startPoseRedWallrider).plus(new Pose2d(75, 6.4, Math.toRadians(-87))), Math.toRadians(0))
                .splineToLinearHeading((startPoseRedWallrider).plus(new Pose2d(80, 14, Math.toRadians(-87))), Math.toRadians(90))
                .splineToLinearHeading((startPoseRedWallrider).plus(new Pose2d(88, 22, Math.toRadians(-87))), Math.toRadians(0))
                .build()
        );
        wallriderRobotBlue.followTrajectorySequence(wallriderRobotBlue.getDrive()
                        .trajectorySequenceBuilder(startPoseBlueWallrider)
                .back(15)
                .turn(Math.toRadians(-90))
                .back(5)
                .splineToLinearHeading((startPoseBlueWallrider).plus(new Pose2d(-9, -34, Math.toRadians(-90))), Math.toRadians(-90))
                        .waitSeconds(1)
                .back(4)
                .turn(Math.toRadians(-180))
                .waitSeconds(0.2)
                .strafeRight(5)
                .splineToLinearHeading((startPoseBlueWallrider).plus(new Pose2d(-18, -15, Math.toRadians(95))), Math.toRadians(180))
                .waitSeconds(0.2)
                .strafeLeft(17)
                .waitSeconds(0.1)
                .strafeRight(3)
                .waitSeconds(0.1)
                .forward(3)
                        .waitSeconds(1)
                .back(8)
                .turn(Math.toRadians(5))
                .splineToLinearHeading((startPoseBlueWallrider).plus(new Pose2d(-18, -10, Math.toRadians(95))), Math.toRadians(90))
                .back(1)
                .waitSeconds(0.1)
                .splineToLinearHeading((startPoseBlueWallrider).plus(new Pose2d(-7, -2, Math.toRadians(95))), Math.toRadians(0))
                .splineToLinearHeading((startPoseBlueWallrider).plus(new Pose2d(40, 2, Math.toRadians(95))), Math.toRadians(0))
                .splineToLinearHeading((startPoseBlueWallrider).plus(new Pose2d(75, 0, Math.toRadians(95))), Math.toRadians(0))
                .splineToLinearHeading((startPoseBlueWallrider).plus(new Pose2d(80, -10, Math.toRadians(95))), Math.toRadians(-90))
                .splineToLinearHeading((startPoseBlueWallrider).plus(new Pose2d(88, -14, Math.toRadians(95))), Math.toRadians(0))
                .build());
        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(closeRobotBlue)
                .addEntity(farRobotBlue)
                .addEntity(closeRobotRed)
                .addEntity(farRobotRed)
                .addEntity(wallriderRobotRed)
                .addEntity(wallriderRobotBlue)
                .start();
    }
}