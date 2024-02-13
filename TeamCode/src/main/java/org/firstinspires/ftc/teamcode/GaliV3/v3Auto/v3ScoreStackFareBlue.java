package org.firstinspires.ftc.teamcode.GaliV3.v3Auto;

import static org.firstinspires.ftc.teamcode.Vision.BlueColorProcessor.centerBlueRatio;
import static org.firstinspires.ftc.teamcode.Vision.BlueColorProcessor.leftBlueRatio;
import static org.firstinspires.ftc.teamcode.Vision.BlueColorProcessor.rightBlueRatio;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.GaliV3.v3Hardware;
import org.firstinspires.ftc.teamcode.GaliV3.v3Roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.GaliV3.v3Roadrunner.trajectorysequence.TrajectorySequence;
@Autonomous

public class v3ScoreStackFareBlue extends v3autoBase{
    Pose2d startPose = new Pose2d(0, 0, 0);
    public static double forWard = 0;
    public static double turn1 = 0;
    double armTime = 0;
    double armTime2 = 0;
    @Override
    public void runOpMode() {
        super.runOpMode();
        cameraStartup("Webcam 1");
        propDetection("blue");
        propPos("blue", "far");
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        robot.flipper.setPosition(v3Hardware.flipUp);
        robot.intake.setPower(0);
        robot.door.setPosition(v3Hardware.doorClosed);
        TrajectorySequence center1 = drive.trajectorySequenceBuilder(startPose)
                .back(10)
                .turn(Math.toRadians(-90))
                .splineToLinearHeading((new Pose2d(-10, 7, Math.toRadians(-90))),Math.toRadians(90))
                .splineToLinearHeading((new Pose2d(-48.5, 7, Math.toRadians(-90))),Math.toRadians(-90))
                .splineToLinearHeading((new Pose2d(-48.5, -1, Math.toRadians(-90))),Math.toRadians(-90))
                .splineToLinearHeading((new Pose2d(-35.5, -5, Math.toRadians(-90))),Math.toRadians(0))
                .back(10)
                .build();
        TrajectorySequence center2 = drive.trajectorySequenceBuilder(center1.end())
                .back(4)
                .addDisplacementMarker(()->{
                    robot.intake.setPower(-v3Hardware.intakeSpeed/8);
                    sleep(1500);
                    robot.intake.setPower(0);
                })
                .waitSeconds(0.5)
                .turn(Math.toRadians(190))
                .splineToLinearHeading((startPose).plus(new Pose2d(-50.5, 7, Math.toRadians(100))),Math.toRadians(180))
                .splineToLinearHeading((startPose).plus(new Pose2d(-50.5, 14, Math.toRadians(100))),Math.toRadians(90))
                .build();
        TrajectorySequence center3 = drive.trajectorySequenceBuilder(center2.end())
                .forward(4.3)
                .build();
        TrajectorySequence center4 = drive.trajectorySequenceBuilder(center3.end())
                .back(5)
                .addDisplacementMarker(()->{
                    robot.intake.setPower(v3Hardware.intakeSpeed);
                    robot.flipper.setPosition(v3Hardware.flipDown);
                    sleep(1000);
                    robot.intake.setPower(-v3Hardware.intakeSpeed);
                    sleep(1000);
                    robot.intake.setPower(0);
                })
                .turn(Math.toRadians(-5))
                .splineToLinearHeading((startPose).plus(new Pose2d(-50.5, -5, Math.toRadians(90))),Math.toRadians(90))
                .splineToLinearHeading((startPose).plus(new Pose2d(-50.5, -80, Math.toRadians(90))),Math.toRadians(-90))
                .splineToLinearHeading((startPose).plus(new Pose2d(-21.5, -85, Math.toRadians(90))),Math.toRadians(0))
                .splineToLinearHeading((startPose).plus(new Pose2d(-21.5, -95, Math.toRadians(90))),Math.toRadians(-90))
                .build();
        TrajectorySequence left1 = drive.trajectorySequenceBuilder(startPose)
                .back(5)
                .turn(Math.toRadians(-90))
                .splineToLinearHeading((startPose).plus(new Pose2d(-26, 13, Math.toRadians(-90))),Math.toRadians(-90-90))
                .forward(24)
                .waitSeconds(0.5)
                .back(9.5)
                .build();
        TrajectorySequence left2 = drive.trajectorySequenceBuilder(left1.end())
                .back(5)
                .turn(Math.toRadians(180))
                .splineToLinearHeading((startPose).plus(new Pose2d(-50, 12, Math.toRadians(90))),Math.toRadians(180-90))
                .build();
        TrajectorySequence left3= drive.trajectorySequenceBuilder(left2.end())
                .forward(7)
                .build();
        TrajectorySequence left4= drive.trajectorySequenceBuilder(left3.end())
                .back(5)
                .addDisplacementMarker(()->{
                    robot.flipper.setPosition(v3Hardware.flipDown);
                    sleep(500);
                    robot.intake.setPower(-v3Hardware.intakeSpeed);
                    sleep(1000);
                    robot.intake.setPower(0);
                })
                .waitSeconds(1)
                .splineToLinearHeading((startPose).plus(new Pose2d(-50, -80, Math.toRadians(90))),Math.toRadians(-90))
                .splineToLinearHeading((startPose).plus(new Pose2d(-19, -86, Math.toRadians(90))),Math.toRadians(0))
                .splineToLinearHeading((startPose).plus(new Pose2d(-19, -90, Math.toRadians(90))),Math.toRadians(-90))
                .build();
        TrajectorySequence right1 = drive.trajectorySequenceBuilder(startPose)
                .back(5)
                .splineToLinearHeading((startPose).plus(new Pose2d(-12, 5, Math.toRadians(180))),Math.toRadians(-90-90))
                .build();
        TrajectorySequence right2 = drive.trajectorySequenceBuilder(right1.end())
                .splineToLinearHeading((startPose).plus(new Pose2d(-10, -4, Math.toRadians(180))),Math.toRadians(-90-90))
                .splineToLinearHeading((startPose).plus(new Pose2d(-55, -4, Math.toRadians(180))),Math.toRadians(-90-90))
                .turn(Math.toRadians(-90))
                .splineToLinearHeading((startPose).plus(new Pose2d(-50, 16, Math.toRadians(90))),Math.toRadians(180-90))
                .build();
        TrajectorySequence right3 = drive.trajectorySequenceBuilder(right2.end())
                .forward(6)
                .build();
        TrajectorySequence right4 = drive.trajectorySequenceBuilder(right3.end())
                .back(5)
                .addDisplacementMarker(()->{
                    robot.intake.setPower(v3Hardware.intakeSpeed);
                    robot.flipper.setPosition(v3Hardware.flipDown);
                    sleep(1500);
                    robot.intake.setPower(-v3Hardware.intakeSpeed);
                    sleep(1000);
                    robot.intake.setPower(0);
                })
                .waitSeconds(1)
                .splineToLinearHeading((startPose).plus(new Pose2d(-59, -65, Math.toRadians(90))),Math.toRadians(0-90))
                .splineToLinearHeading((startPose).plus(new Pose2d(-40, -90, Math.toRadians(90))),Math.toRadians(0-90))
                .splineToLinearHeading((startPose).plus(new Pose2d(-34, -97, Math.toRadians(90))),Math.toRadians(0-90))
                .build();
        while (!isStarted()) {
            telemetry.addData("position", propPos("blue", "far"));
            telemetry.addData("rightBlue", rightBlueRatio);
            telemetry.addData("centerBlue", centerBlueRatio);
            telemetry.addData("leftBlue", leftBlueRatio);
            telemetry.addData("distance",robot.distanceTop.getDistance(DistanceUnit.MM));
            telemetry.update();
        }
        waitForStart();
        robot.intake.setPower(-0.02);
        /*Middle
        drive.followTrajectorySequence(center1);
        telemetry.addData("distance",robot.distanceTop.getDistance(DistanceUnit.MM));
        telemetry.update();
        robot.intake.setPower(0);
        robot.flipper.setPosition(v3Hardware.flipDown);
        sleep(500);
        robot.intake.setPower(-0.2);
        sleep(500);
        robot.intake.setPower(0);
        robot.flipper.setPosition(v3Hardware.flipUp);
        drive.followTrajectorySequence(center2);
        robot.intake.setPower(v3Hardware.intakeSpeed);
        robot.flipper.setPosition(v3Hardware.flipUp-((v3Hardware.flipUp-v3Hardware.flipDown)/1.8));
        drive.followTrajectorySequence(center3);
        sleep(500);
        drive.turn(Math.toRadians(10));
        sleep(500);
        drive.turn(Math.toRadians(-10));
        robot.intake.setPower(0);
        robot.flipper.setPosition(v3Hardware.flipUp);
        drive.followTrajectorySequence(center4);
        robot.shoulderStar.setPosition(v3Hardware.shoulderStarScore);
        robot.shoulderPort.setPosition(v3Hardware.shoulderPortScore);
        robot.portArm.setPower(1);
        robot.starArm.setPower(1);
        sleep(500);
        robot.portArm.setPower(0);
        robot.starArm.setPower(0);
        sleep(2000);
        robot.door.setPosition(v3Hardware.doorOpen);
        sleep(1000);
        robot.elbow.setPosition(v3Hardware.elbowNorminal);
        robot.shoulderStar.setPosition(v3Hardware.shoulderStarScore-0.05);
        robot.shoulderPort.setPosition(v3Hardware.shoulderPortScore+0.05);
        sleep(300);
        robot.portArm.setPower(-1);
        robot.starArm.setPower(-1);
        robot.shoulderStar.setPosition(v3Hardware.shoulderStarDown);
        robot.shoulderPort.setPosition(v3Hardware.shoulderPortDown);
        sleep(2000);
         */
        if(propPos.equals("right")){
        drive.followTrajectorySequence(right1);
        telemetry.addData("distance",robot.distanceTop.getDistance(DistanceUnit.MM));
        telemetry.update();
        robot.intake.setPower(0);
        robot.flipper.setPosition(v3Hardware.flipDown);
        sleep(500);
        robot.intake.setPower(-0.2);
        sleep(500);
        robot.intake.setPower(0);
        robot.flipper.setPosition(v3Hardware.flipUp);
        drive.followTrajectorySequence(right2);
        robot.intake.setPower(v3Hardware.intakeSpeed);
        robot.flipper.setPosition(v3Hardware.flipUp-((v3Hardware.flipUp-v3Hardware.flipDown)/1.8));
        drive.followTrajectorySequence(right3);
        sleep(500);
        drive.turn(Math.toRadians(10));
        sleep(500);
        drive.turn(Math.toRadians(-10));
        robot.intake.setPower(0);
        robot.flipper.setPosition(v3Hardware.flipUp);
        drive.followTrajectorySequence(right4);
        robot.shoulderStar.setPosition(v3Hardware.shoulderStarScore);
        robot.shoulderPort.setPosition(v3Hardware.shoulderPortScore);
        robot.portArm.setPower(1);
        robot.starArm.setPower(1);
        sleep(250);
        robot.portArm.setPower(0);
        robot.starArm.setPower(0);
        sleep(2000);
        resetRuntime();
        while (getRuntime() < 3 && !robot.handTS.isPressed()) {
            drive.setMotorPowers(-0.2, -0.2, -0.2, -0.2);
        }
        drive.setMotorPowers(-0, -0, -0, -0);
        robot.door.setPosition(v3Hardware.doorOpen);
        sleep(1000);
        robot.elbow.setPosition(v3Hardware.elbowNorminal);
        robot.shoulderStar.setPosition(v3Hardware.shoulderStarScore-0.05);
        robot.shoulderPort.setPosition(v3Hardware.shoulderPortScore+0.05);
        sleep(250);
        robot.portArm.setPower(-1);
        robot.starArm.setPower(-1);
        robot.shoulderStar.setPosition(v3Hardware.shoulderStarDown);
        robot.shoulderPort.setPosition(v3Hardware.shoulderPortDown);
        robot.door.setPosition(v3Hardware.doorClosed);
        sleep(2000);
    }
        else if(propPos.equals("center")){
        drive.followTrajectorySequence(center1);
        telemetry.addData("distance",robot.distanceTop.getDistance(DistanceUnit.MM));
        telemetry.update();
        robot.intake.setPower(0);
        robot.flipper.setPosition(v3Hardware.flipDown);
        sleep(500);
        robot.intake.setPower(-0.2);
        sleep(500);
        robot.intake.setPower(0);
        robot.flipper.setPosition(v3Hardware.flipUp);
        drive.followTrajectorySequence(center2);
        robot.intake.setPower(v3Hardware.intakeSpeed);
        robot.flipper.setPosition(v3Hardware.flipUp-((v3Hardware.flipUp-v3Hardware.flipDown)/1.8));
        drive.followTrajectorySequence(center3);
        sleep(500);
        drive.turn(Math.toRadians(10));
        sleep(500);
        drive.turn(Math.toRadians(-10));
        robot.intake.setPower(0);
        robot.flipper.setPosition(v3Hardware.flipUp);
        drive.followTrajectorySequence(center4);
        robot.shoulderStar.setPosition(v3Hardware.shoulderStarScore);
        robot.shoulderPort.setPosition(v3Hardware.shoulderPortScore);
        robot.portArm.setPower(1);
        robot.starArm.setPower(1);
        sleep(250);
        robot.portArm.setPower(0);
        robot.starArm.setPower(0);
        sleep(2000);
        resetRuntime();
        while (getRuntime() < 3 && !robot.handTS.isPressed()) {
            drive.setMotorPowers(-0.2, -0.2, -0.2, -0.2);
        }
        drive.setMotorPowers(-0, -0, -0, -0);
        robot.door.setPosition(v3Hardware.doorOpen);
        sleep(1000);
        robot.elbow.setPosition(v3Hardware.elbowNorminal);
        robot.shoulderStar.setPosition(v3Hardware.shoulderStarScore-0.05);
        robot.shoulderPort.setPosition(v3Hardware.shoulderPortScore+0.05);
        sleep(300);
        robot.portArm.setPower(-1);
        robot.starArm.setPower(-1);
        robot.shoulderStar.setPosition(v3Hardware.shoulderStarDown);
        robot.shoulderPort.setPosition(v3Hardware.shoulderPortDown);
        robot.door.setPosition(v3Hardware.doorClosed);
        sleep(2000);
    }
        else{
        drive.followTrajectorySequence(left1);
        telemetry.addData("distance",robot.distanceTop.getDistance(DistanceUnit.MM));
        telemetry.update();
        robot.intake.setPower(0);
        robot.flipper.setPosition(v3Hardware.flipDown);
        sleep(500);
        robot.intake.setPower(-0.2);
        sleep(500);
        robot.intake.setPower(0);
        robot.flipper.setPosition(v3Hardware.flipUp);
        drive.followTrajectorySequence(left2);
        robot.intake.setPower(v3Hardware.intakeSpeed);
        robot.flipper.setPosition(v3Hardware.flipUp-((v3Hardware.flipUp-v3Hardware.flipDown)/1.8));
        drive.followTrajectorySequence(left3);
        sleep(500);
        drive.turn(Math.toRadians(10));
        sleep(500);
        drive.turn(Math.toRadians(-10));
        robot.intake.setPower(0);
        robot.flipper.setPosition(v3Hardware.flipUp);
        drive.followTrajectorySequence(left4);
        robot.shoulderStar.setPosition(v3Hardware.shoulderStarScore);
        robot.shoulderPort.setPosition(v3Hardware.shoulderPortScore);
        robot.portArm.setPower(1);
        robot.starArm.setPower(1);
        sleep(300);
        robot.portArm.setPower(0);
        robot.starArm.setPower(0);
        sleep(2000);
        resetRuntime();
        while (getRuntime() < 3 && !robot.handTS.isPressed()) {
            drive.setMotorPowers(-0.2, -0.2, -0.2, -0.2);
        }
        drive.setMotorPowers(-0, -0, -0, -0);
        robot.door.setPosition(v3Hardware.doorOpen);
        sleep(1000);
        robot.elbow.setPosition(v3Hardware.elbowNorminal);
        robot.shoulderStar.setPosition(v3Hardware.shoulderStarScore-0.05);
        robot.shoulderPort.setPosition(v3Hardware.shoulderPortScore+0.05);
        sleep(300);
        robot.portArm.setPower(-1);
        robot.starArm.setPower(-1);
        robot.shoulderStar.setPosition(v3Hardware.shoulderStarDown);
        robot.shoulderPort.setPosition(v3Hardware.shoulderPortDown);
        robot.door.setPosition(v3Hardware.doorClosed);
        sleep(2000);
            }
    }
        }
