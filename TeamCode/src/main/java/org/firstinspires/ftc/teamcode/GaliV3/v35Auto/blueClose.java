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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.GaliV3.v3Auto.v3autoBase;
import org.firstinspires.ftc.teamcode.GaliV3.v3Hardware;
import org.firstinspires.ftc.teamcode.GaliV3.v3Roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.GaliV3.v3Roadrunner.trajectorysequence.TrajectorySequence;
@Disabled
@Autonomous
public class blueClose   extends v3autoBase {
    Pose2d startPose = new Pose2d(-15, 61, Math.toRadians(90));
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
        propPos("blue", "close");
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        robot.flipper.setPosition(v3Hardware.flipUp);
        robot.intake.setPower(0);
        robot.door.setPosition(v3Hardware.doorClosed);
        TrajectorySequence left1 = drive.trajectorySequenceBuilder(startPose)
                .back(4, getVelocityConstraint(MAX_VEL/1.5,MAX_ANG_VEL,TRACK_WIDTH), getAccelerationConstraint(MAX_ACCEL/1.5))
                .turn(Math.toRadians(180))
                .splineToLinearHeading((startPose).plus(new Pose2d(14, -15, Math.toRadians(180))), Math.toRadians(-90), getVelocityConstraint(MAX_VEL/1.5,MAX_ANG_VEL,TRACK_WIDTH), getAccelerationConstraint(MAX_ACCEL/1.5))
                .build();
        TrajectorySequence left2 = drive.trajectorySequenceBuilder(left1.end())
                .addDisplacementMarker(()-> {
                    robot.shoulderPort.setPosition(v3Hardware.shoulderPortScore);
                    robot.shoulderStar.setPosition(v3Hardware.shoulderStarScore);
                })
                .strafeLeft(7)
                .splineToLinearHeading((startPose).plus(new Pose2d(32, -23, Math.toRadians(90))), Math.toRadians(0))
                .build();
        TrajectorySequence left3 = drive.trajectorySequenceBuilder(left2.end())
                .splineToLinearHeading((startPose).plus(new Pose2d(30, -1, Math.toRadians(85))), Math.toRadians(0))
                .back(13)
                .build();
        TrajectorySequence center1 = drive.trajectorySequenceBuilder(startPose)
                .back(8)
                .turn(Math.toRadians(90))
                .back(1)
                .splineToLinearHeading((startPose).plus(new Pose2d(24, -12, Math.toRadians(90))), Math.toRadians(0))
                .splineToLinearHeading((startPose).plus(new Pose2d(24, -40, Math.toRadians(90))), Math.toRadians(-90))
                .splineToLinearHeading((startPose).plus(new Pose2d(16, -45, Math.toRadians(90))), Math.toRadians(0))
                .build();
        TrajectorySequence center2 = drive.trajectorySequenceBuilder(center1.end())
                .addDisplacementMarker(()-> {
                    robot.shoulderPort.setPosition(v3Hardware.shoulderPortScore);
                    robot.shoulderStar.setPosition(v3Hardware.shoulderStarScore);
                })
                .back(1)
                .splineToLinearHeading((startPose).plus(new Pose2d(33, -26, Math.toRadians(90))), Math.toRadians(0))
                .build();
        TrajectorySequence center3 = drive.trajectorySequenceBuilder(center2.end())
                .splineToLinearHeading((startPose).plus(new Pose2d(30, -1, Math.toRadians(90))), Math.toRadians(0))
                .back(13)
                .build();
        TrajectorySequence right1 = drive.trajectorySequenceBuilder(startPose)
                .back(29)
                .turn(Math.toRadians(90))
                .forward(15)
                .back(10)
                .build();
        TrajectorySequence right2 = drive.trajectorySequenceBuilder(right1.end())
                .addDisplacementMarker(()-> {
                    robot.shoulderPort.setPosition(v3Hardware.shoulderPortScore);
                    robot.shoulderStar.setPosition(v3Hardware.shoulderStarScore);
                })
                .back(10)
                .splineToLinearHeading((startPose).plus(new Pose2d(33, -32, Math.toRadians(90))), Math.toRadians(0))
                .build();
        TrajectorySequence right3 = drive.trajectorySequenceBuilder(right2.end())
                .splineToLinearHeading((startPose).plus(new Pose2d(30, -1, Math.toRadians(90))), Math.toRadians(0))
                .back(13)
                .build();

        while (!isStarted()) {
            telemetry.addData("position", propPos("blue", "close"));
            telemetry.addData("rightBlue", rightBlueRatio);
            telemetry.addData("centerBlue", centerBlueRatio);
            telemetry.addData("leftBlue", leftBlueRatio);
            telemetry.addData("distance", robot.distanceTop.getDistance(DistanceUnit.MM));
            propPos =propPos("blue", "close");
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
        else if(propPos.equals("left")) {
            drive.followTrajectorySequence(left2);}
        else{
            drive.followTrajectorySequence(right2);
        }
        resetRuntime();
        while(!robot.handTS.isPressed()&& getRuntime()<3){
            drive.setMotorPowers(-0.3,-0.3,-0.3,-0.3);
        }
        drive.setMotorPowers(0,0,0,0);
        sleep(300);
        robot.door.setPosition(v3Hardware.doorOpen);
        sleep(1000);
        if(propPos.equals("center")){
            drive.followTrajectorySequence(center3);
        }
        else if(propPos.equals("left")){
            drive.followTrajectorySequence(left3);
        }
        else{
            drive.followTrajectorySequence(right3);
        }


    }}