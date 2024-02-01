package org.firstinspires.ftc.teamcode.GaliV3.v3Auto;
import static org.firstinspires.ftc.teamcode.GaliV3.v3Roadrunner.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.GaliV3.v3Roadrunner.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.GaliV3.v3Roadrunner.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.GaliV3.v3Roadrunner.drive.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.GaliV3.v3Roadrunner.drive.SampleMecanumDrive.getAccelerationConstraint;
import static org.firstinspires.ftc.teamcode.GaliV3.v3Roadrunner.drive.SampleMecanumDrive.getVelocityConstraint;
import static org.firstinspires.ftc.teamcode.Vision.RedColorProcessor.centerRedRatio;
import static org.firstinspires.ftc.teamcode.Vision.RedColorProcessor.leftRedRatio;
import static org.firstinspires.ftc.teamcode.Vision.RedColorProcessor.rightRedRatio;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.GaliV3.v3Hardware;
import org.firstinspires.ftc.teamcode.GaliV3.v3Roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.GaliV3.v3Roadrunner.trajectorysequence.TrajectorySequence;

import java.util.Objects;
@Autonomous
public class v3OnTakeScoreFareRed extends v3autoBase{
    Pose2d startPose = new Pose2d(0, 0, 0);
    public static double forWard = 0;
    public static double turn1 = 0;
    String pos = "notSeen";
    @Override
    public void runOpMode() {
        super.runOpMode();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        cameraStartup("Webcam 1");
        propDetection("red");
        propPos("red", "far");
        TrajectorySequence center1 = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-10, -13, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(-20, -13, Math.toRadians(60)))
                .lineToLinearHeading(new Pose2d(-30, -13, Math.toRadians(85)))
                .lineToLinearHeading(new Pose2d(-41, -13, Math.toRadians(85)))
                .build();
        TrajectorySequence center2 = drive.trajectorySequenceBuilder(center1.end())
                .lineToLinearHeading(new Pose2d(-50, -14, Math.toRadians(85)),getVelocityConstraint(MAX_VEL/1.5, MAX_ANG_VEL/1.2, TRACK_WIDTH),getAccelerationConstraint(MAX_ACCEL/1.5))
                .lineToLinearHeading(new Pose2d(-52, -14, Math.toRadians(-85)),getVelocityConstraint(MAX_VEL/1.5, MAX_ANG_VEL/1.2, TRACK_WIDTH),getAccelerationConstraint(MAX_ACCEL/1.5))
                .lineToLinearHeading(new Pose2d(-49, 50, Math.toRadians(-95)),getVelocityConstraint(MAX_VEL/1.5, MAX_ANG_VEL/1.2, TRACK_WIDTH),getAccelerationConstraint(MAX_ACCEL/1.5))
                .lineToLinearHeading(new Pose2d(-37.5, 87, Math.toRadians(-97)),getVelocityConstraint(MAX_VEL/1.5, MAX_ANG_VEL/1.2, TRACK_WIDTH),getAccelerationConstraint(MAX_ACCEL/1.5))
                .build();
        TrajectorySequence center3 = drive.trajectorySequenceBuilder(center2.end())
                .lineToLinearHeading(new Pose2d(-37.5, 90, Math.toRadians(-95)))
                .build();
        TrajectorySequence left1 = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-10, -8, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(-11, -8, Math.toRadians(-180)))
                .lineToLinearHeading(new Pose2d(-20, -8, Math.toRadians(-180)))
                .build();
        TrajectorySequence left2 = drive.trajectorySequenceBuilder(left1.end())
                .lineToLinearHeading(new Pose2d(-10, -9, Math.toRadians(-180)),getVelocityConstraint(MAX_VEL/1.3, MAX_ANG_VEL/1.2, TRACK_WIDTH),getAccelerationConstraint(MAX_ACCEL/1.5))
                .lineToLinearHeading(new Pose2d(-10, 1, Math.toRadians(-180)),getVelocityConstraint(MAX_VEL/1.3, MAX_ANG_VEL/1.2, TRACK_WIDTH),getAccelerationConstraint(MAX_ACCEL/1.5))
                .lineToLinearHeading(new Pose2d(-50, 1, Math.toRadians(-180)),getVelocityConstraint(MAX_VEL/1.3, MAX_ANG_VEL/1.2, TRACK_WIDTH),getAccelerationConstraint(MAX_ACCEL/1.5))
                .lineToLinearHeading(new Pose2d(-50, -5, Math.toRadians(-110)),getVelocityConstraint(MAX_VEL/1.3, MAX_ANG_VEL/1.2, TRACK_WIDTH),getAccelerationConstraint(MAX_ACCEL/1.5))
                .lineToLinearHeading(new Pose2d(-56, -5, Math.toRadians(-80)),getVelocityConstraint(MAX_VEL/1.3, MAX_ANG_VEL/1.2, TRACK_WIDTH),getAccelerationConstraint(MAX_ACCEL/1.5))
                .lineToLinearHeading(new Pose2d(-57, 50, Math.toRadians(-80)),getVelocityConstraint(MAX_VEL/1.3, MAX_ANG_VEL/1.2, TRACK_WIDTH),getAccelerationConstraint(MAX_ACCEL/1.5))
                .lineToLinearHeading(new Pose2d(-58, 86, Math.toRadians(-85)),getVelocityConstraint(MAX_VEL/1.3, MAX_ANG_VEL/1.2, TRACK_WIDTH),getAccelerationConstraint(MAX_ACCEL/1.5))
                .build();
        TrajectorySequence left3 = drive.trajectorySequenceBuilder(left2.end())
                .lineToLinearHeading(new Pose2d(-59, 90, Math.toRadians(-80)))
                .build();
        TrajectorySequence right1 = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-25, 0, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(-26, 0, Math.toRadians(45)))
                .lineToLinearHeading(new Pose2d(-27, 0, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(-28, 20, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(-30, 5.5, Math.toRadians(90)))
                .build();
        TrajectorySequence right2 = drive.trajectorySequenceBuilder(right1.end())
                .lineToLinearHeading(new Pose2d(-28, 0, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(-29, 0, Math.toRadians(0)),getVelocityConstraint(MAX_VEL/2.2, MAX_ANG_VEL/1.5, TRACK_WIDTH),getAccelerationConstraint(MAX_ACCEL/1.9))
                .lineToLinearHeading(new Pose2d(-50, -4, Math.toRadians(0)),getVelocityConstraint(MAX_VEL/2.2, MAX_ANG_VEL/1.5, TRACK_WIDTH),getAccelerationConstraint(MAX_ACCEL/1.9))
                .lineToLinearHeading(new Pose2d(-54, -4, Math.toRadians(-93)),getVelocityConstraint(MAX_VEL/2.2, MAX_ANG_VEL/1.5, TRACK_WIDTH),getAccelerationConstraint(MAX_ACCEL/1.9))
                .lineToLinearHeading(new Pose2d(-54, 20, Math.toRadians(-93)),getVelocityConstraint(MAX_VEL/2.2, MAX_ANG_VEL/1.5, TRACK_WIDTH),getAccelerationConstraint(MAX_ACCEL/1.9))
                .lineToLinearHeading(new Pose2d(-54, 80, Math.toRadians(-93)),getVelocityConstraint(MAX_VEL/2.2, MAX_ANG_VEL/1.5, TRACK_WIDTH),getAccelerationConstraint(MAX_ACCEL/1.9))
                .lineToLinearHeading(new Pose2d(-37, 86, Math.toRadians(-93)),getVelocityConstraint(MAX_VEL/2.2, MAX_ANG_VEL/1.5, TRACK_WIDTH),getAccelerationConstraint(MAX_ACCEL/1.9))
                .build();
        TrajectorySequence right3 = drive.trajectorySequenceBuilder(right2.end())
                .lineToLinearHeading(new Pose2d(-37, 90, Math.toRadians(-85)))
                .build();


        while(!isStarted()) {
            telemetry.addData("position", pos);
            telemetry.addData("leftRed", leftRedRatio);
            telemetry.addData("centerRed", centerRedRatio);
            telemetry.addData("rightRed", rightRedRatio);
            pos = propPos("red", "far");
            telemetry.update();
        }
        waitForStart();
       if(pos.equals("center")) {
           drive.followTrajectorySequence(center1);
           sleep(1000);
           robot.intake.setPower(-0.5);
           sleep(800);
           robot.intake.setPower(0);
           sleep(3000);
           drive.followTrajectorySequence(center2);
           scoreBack();
           sleep(1000);
           resetRuntime();
           while (getRuntime() < 0.7) {
               robot.portArm.setPower(1);
               robot.starArm.setPower(1);
           }
           robot.starArm.setPower(0);
           robot.portArm.setPower(0);
           sleep(1000);
           scorePort();
           sleep(2000);
           drive.followTrajectorySequence(center3);
           sleep(1000);
           robot.door.setPosition(v3Hardware.doorOpen);
           while (getRuntime() < 0.3) {
               robot.portArm.setPower(1);
               robot.starArm.setPower(1);
           }
           sleep(1000);
           resetArm();
           sleep(5000);}
        //V3 left side
        if(pos.equals("left")) {
            drive.followTrajectorySequence(left1);
            sleep(1000);
            robot.intake.setPower(-0.5);
            sleep(600);
            robot.intake.setPower(0);
            drive.followTrajectorySequence(left2);
            scoreBack();
            sleep(1000);
            resetRuntime();
            while (getRuntime() < 0.7) {
                robot.portArm.setPower(1);
                robot.starArm.setPower(1);
            }
            robot.starArm.setPower(0);
            robot.portArm.setPower(0);
            sleep(1000);
            scorePort();
            sleep(2000);
            drive.followTrajectorySequence(left3);
            sleep(1000);
            robot.door.setPosition(v3Hardware.doorOpen);
            while (getRuntime() < 0.3) {
                robot.portArm.setPower(1);
                robot.starArm.setPower(1);
            }
            sleep(1000);
            resetArm();
            sleep(5000);
        }
        if(pos.equals("right")) {
            drive.followTrajectorySequence(right1);
            sleep(1000);
            robot.intake.setPower(-0.5);
            sleep(600);
            robot.intake.setPower(0);

            drive.followTrajectorySequence(right2);
            scoreBack();
            sleep(1000);
            resetRuntime();
            while (getRuntime() < 0.95) {
                robot.portArm.setPower(1);
                robot.starArm.setPower(1);
            }
            robot.starArm.setPower(0);
            robot.portArm.setPower(0);
            sleep(1000);
            scorePort();
            sleep(2000);
            drive.followTrajectorySequence(right3);
            sleep(1000);
            robot.door.setPosition(v3Hardware.doorOpen);
            while (getRuntime() < 0.3) {
                robot.portArm.setPower(1);
                robot.starArm.setPower(1);
            }
            sleep(1000);
            resetArm();
            sleep(5000);}}}
