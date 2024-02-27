package org.firstinspires.ftc.teamcode.GaliV3.v35Auto;

import static org.firstinspires.ftc.teamcode.GaliV3.v3Roadrunner.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.GaliV3.v3Roadrunner.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.GaliV3.v3Roadrunner.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.GaliV3.v3Roadrunner.drive.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.GaliV3.v3Roadrunner.drive.SampleMecanumDrive.getAccelerationConstraint;
import static org.firstinspires.ftc.teamcode.GaliV3.v3Roadrunner.drive.SampleMecanumDrive.getVelocityConstraint;
import static org.firstinspires.ftc.teamcode.Vision.BlueColorProcessor.centerBlueRatio;
import static org.firstinspires.ftc.teamcode.Vision.BlueColorProcessor.leftBlueRatio;
import static org.firstinspires.ftc.teamcode.Vision.BlueColorProcessor.rightBlueRatio;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.GaliV3.v3Auto.v3autoBase;
import org.firstinspires.ftc.teamcode.GaliV3.v3Hardware;
import org.firstinspires.ftc.teamcode.GaliV3.v3Roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.GaliV3.v3Roadrunner.trajectorysequence.TrajectorySequence;
@Autonomous
public class blueFareAuto  extends v3autoBase {
    Pose2d startPose = new Pose2d(-38, 61, Math.toRadians(90));
    public static double forWard = 0;
    public static double turn1 = 0;
    double armTime = 0;
    double armTime2 = 0;
    String distanceString = "not seen";
    double distanceTimer = -3;
    @Override
    public void runOpMode() {
        super.runOpMode();
        robot.intake.setPower(0.02);
        cameraStartup("Webcam 1");
        propDetection("blue");
        propPos("blue", "far");
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        robot.flipper.setPosition(v3Hardware.flipUp);
        robot.intake.setPower(0);
        robot.door.setPosition(v3Hardware.doorClosed);

        TrajectorySequence center1 = drive.trajectorySequenceBuilder(startPose)
                .back(15)
                .turn(Math.toRadians(-90))
                .back(5)
                .splineToLinearHeading((startPose).plus(new Pose2d(-8, -34, Math.toRadians(-90))), Math.toRadians(-90))
                .build();
        TrajectorySequence center2 = drive.trajectorySequenceBuilder(center1.end())
                .back(5)
                .turn(Math.toRadians(-180))
                .waitSeconds(0.2)
                .strafeRight(5)
                .addDisplacementMarker(()->{
                    robot.flipper.setPosition(v3Hardware.flipDown);
                })
                .splineToLinearHeading((startPose).plus(new Pose2d(-18, -15, Math.toRadians(95))), Math.toRadians(180))
                .waitSeconds(0.2)
                .strafeLeft(17)
                .waitSeconds(0.1)
                .strafeRight(3)
                .waitSeconds(0.1)
                .forward(3)
                .build();
        TrajectorySequence center3 = drive.trajectorySequenceBuilder(center2.end())
                .addDisplacementMarker(()->{
                    robot.flipper.setPosition(v3Hardware.flipUp+0.3);
                    robot.intake.setPower(0);
                })
                .turn(Math.toRadians(5))
                .back(3)
                .splineToLinearHeading((startPose).plus(new Pose2d(-17, -51.5, Math.toRadians(100))), Math.toRadians(0))
                .splineToLinearHeading((startPose).plus(new Pose2d(65, -49.5, Math.toRadians(100))), Math.toRadians(0))
                .splineToLinearHeading((startPose).plus(new Pose2d(78, -50, Math.toRadians(100))), Math.toRadians(90))
                .splineToLinearHeading((startPose).plus(new Pose2d(87, -29, Math.toRadians(100))), Math.toRadians(0))
                .build();
        TrajectorySequence right1 = drive.trajectorySequenceBuilder(startPose)
                .back(5)
                .turn(Math.toRadians(180))
                .splineToLinearHeading((startPose).plus(new Pose2d(-1, -17, Math.toRadians(180))),Math.toRadians(-90))
                .build();
        TrajectorySequence right2 = drive.trajectorySequenceBuilder(right1.end())
                .back(5)
                .turn(Math.toRadians(-90))
                .waitSeconds(0.2)
                .strafeRight(5)
                .addDisplacementMarker(()->{
                    robot.flipper.setPosition(v3Hardware.flipDown);
                })
                .splineToLinearHeading((startPose).plus(new Pose2d(-20, -15, Math.toRadians(95))), Math.toRadians(180))
                .waitSeconds(0.2)
                .strafeLeft(17)
                .waitSeconds(0.1)
                .strafeRight(3)
                .waitSeconds(0.1)
                .forward(1)
                .build();
        TrajectorySequence right3 = drive.trajectorySequenceBuilder(right2.end())
                .addDisplacementMarker(()->{
                    robot.flipper.setPosition(v3Hardware.flipUp+0.3);
                    robot.intake.setPower(-v3Hardware.intakeSpeed);
                })
                .turn(Math.toRadians(5))
                .back(2)
                .turn(Math.toRadians(10))
                .strafeLeft(20)
                .turn(Math.toRadians(-20))
                .splineToLinearHeading((startPose).plus(new Pose2d(-13, -51.5, Math.toRadians(90))), Math.toRadians(0))
                .splineToLinearHeading((startPose).plus(new Pose2d(65, -49.5, Math.toRadians(90))), Math.toRadians(0))
                .splineToLinearHeading((startPose).plus(new Pose2d(78, -52, Math.toRadians(90))), Math.toRadians(0))
                .splineToLinearHeading((startPose).plus(new Pose2d(87, -40, Math.toRadians(90))), Math.toRadians(0))
                .build();
        TrajectorySequence left1 = drive.trajectorySequenceBuilder(startPose)
                .back(27)
                .turn(Math.toRadians(-90))
                .forward(10,getVelocityConstraint(MAX_VEL/1.3,MAX_ANG_VEL,TRACK_WIDTH), getAccelerationConstraint(MAX_ACCEL))
                .back(5,getVelocityConstraint(MAX_VEL/1.3,MAX_ANG_VEL,TRACK_WIDTH), getAccelerationConstraint(MAX_ACCEL))
                .build();
        TrajectorySequence left2 = drive.trajectorySequenceBuilder(left1.end())
                .back(5)
                .turn(Math.toRadians(180))
                .waitSeconds(0.1)
                .addDisplacementMarker(()->{
                    robot.flipper.setPosition(v3Hardware.flipDown);
                })
                .splineToLinearHeading((startPose).plus(new Pose2d(-19, -15, Math.toRadians(95))), Math.toRadians(180))
                .waitSeconds(0.1)
                .strafeLeft(17)
                .waitSeconds(0.1)
                .strafeRight(3)
                .waitSeconds(0.1)
                .forward(3)
                .build();
        TrajectorySequence left3 = drive.trajectorySequenceBuilder(left2.end())
                .addDisplacementMarker(()->{
                    robot.flipper.setPosition(v3Hardware.flipUp+0.3);
                    robot.intake.setPower(-v3Hardware.intakeSpeed);
                })
                .turn(Math.toRadians(-5))
                .back(3)
                .splineToLinearHeading((startPose).plus(new Pose2d(-17, -51.5, Math.toRadians(90))), Math.toRadians(0))
                .splineToLinearHeading((startPose).plus(new Pose2d(65, -49.5, Math.toRadians(90))), Math.toRadians(0))
                .splineToLinearHeading((startPose).plus(new Pose2d(78, -50, Math.toRadians(90))), Math.toRadians(90))
                .splineToLinearHeading((startPose).plus(new Pose2d(86, -23.5, Math.toRadians(90))), Math.toRadians(0))
                .build();
        while (!isStarted()) {
            telemetry.addData("position", propPos("blue", "blue"));
            telemetry.addData("rightBlue", rightBlueRatio);
            telemetry.addData("centerBlue", centerBlueRatio);
            telemetry.addData("leftBlue", leftBlueRatio);
            telemetry.addData("distance", robot.distanceTop.getDistance(DistanceUnit.MM));
            propPos =propPos("blue", "blue");
            telemetry.update();
        }

        waitForStart();
        drive.setPoseEstimate(startPose);
        if(propPos.equals("center")) {
            drive.followTrajectorySequence(center1);
        }
        else if(propPos.equals("left")){
            drive.followTrajectorySequence(left1);
        }
        else{
            drive.followTrajectorySequence(right1);
        }
        robot.flipper.setPosition(v3Hardware.flipDown);
        sleep(1000);
        robot.flipper.setPosition(v3Hardware.flipUp);
        if(propPos.equals("center")) {
            drive.followTrajectorySequence(center2);}
        else if(propPos.equals("left")){
            drive.followTrajectorySequence(left2);}
        else{
            drive.followTrajectorySequence(right2);}
        /*
        robot.flipper.setPosition(v3Hardware.flipUp +((v3Hardware.flipDown - v3Hardware.flipUp) / 1.3));
        sleep(500);
        robot.intake.setPower(v3Hardware.intakeSpeed);
        telemetry.addData("topDist",robot.distanceTop.getDistance(DistanceUnit.MM) );
        telemetry.addData("BottomDist",robot.distanceBottom.getDistance(DistanceUnit.MM) );
        telemetry.update();
        resetRuntime();
        while (getRuntime() < 5 && (robot.distanceTop.getDistance(DistanceUnit.MM) > 40 || robot.distanceBottom.getDistance(DistanceUnit.MM) > 40)) {
            robot.flipper.setPosition(v3Hardware.flipUp + (v3Hardware.flipDown - v3Hardware.flipUp) / (1.3 - (getRuntime() / 16)));
            telemetry.addData("topDist",robot.distanceTop.getDistance(DistanceUnit.MM));
            telemetry.addData("BottomDist",robot.distanceBottom.getDistance(DistanceUnit.MM));
            telemetry.update();
            if(getRuntime()>1&&getRuntime()<2){
                drive.setMotorPowers(0.2,0.2,-0.2,-0.2);
            }
            if(getRuntime()>2&&getRuntime()<3){
                drive.setMotorPowers(-0.2,-0.2,0.2,0.2);
            }
        }
        drive.setMotorPowers(0,0,0,0);*/
        robot.intake.setPower(v3Hardware.intakeSpeed);
        robot.flipper.setPosition(v3Hardware.flipDown);
        sleep(1500);
        robot.intake.setPower(-v3Hardware.intakeSpeed/1.5);
        sleep(500);
        robot.intake.setPower(v3Hardware.intakeSpeed);
        sleep(500);
        robot.intake.setPower(-v3Hardware.intakeSpeed/1.5);
        if(propPos.equals("center")) {
            drive.followTrajectorySequence(center3);
            drive.turn(Math.toRadians(-5));
        }
        else if(propPos.equals("left")){
            drive.followTrajectorySequence(left3);
            drive.turn(Math.toRadians(10));
        }
        else{
            drive.followTrajectorySequence(right3);
        }
        robot.intake.setPower(v3Hardware.intakeSpeed);
        robot.shoulderPort.setPosition(v3Hardware.shoulderPortScore);
        robot.shoulderStar.setPosition(v3Hardware.shoulderStarScore);
        sleep(2500);
        /*robot.wrist.setPosition(v3Hardware.wristPort);
        sleep(500);
        robot.wrist.setPosition(v3Hardware.extendyBoiExtend);
        sleep(1000);
        robot.wrist.setPosition(v3Hardware.wristDown);
        sleep(500);*/
        resetRuntime();
        while(!robot.handTS.isPressed()&& getRuntime()<3){
            drive.setMotorPowers(-0.3,-0.3,-0.3,-0.3);
        }
        drive.setMotorPowers(0,0,0,0);
        sleep(300);
        robot.portArm.setPower(0.5);
        robot.starArm.setPower(0.5);
        robot.door.setPosition(v3Hardware.doorOpen);
        robot.portArm.setPower(0);
        robot.starArm.setPower(0);
        sleep(1000);
        /*robot.wrist.setPosition(v3Hardware.wristPort);
        sleep(500);
        robot.wrist.setPosition(v3Hardware.extendyBoiRetract);
        sleep(1000);
        robot.wrist.setPosition(v3Hardware.wristDown);*/
        robot.shoulderPort.setPosition(v3Hardware.shoulderPortDown);
        robot.shoulderStar.setPosition(v3Hardware.shoulderStarDown);
        sleep(2500);
    }
}