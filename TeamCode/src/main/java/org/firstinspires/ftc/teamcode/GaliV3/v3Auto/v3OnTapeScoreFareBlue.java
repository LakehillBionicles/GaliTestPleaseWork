package org.firstinspires.ftc.teamcode.GaliV3.v3Auto;
import static org.firstinspires.ftc.teamcode.GaliV3.v3Hardware.elbowNorminal;
import static org.firstinspires.ftc.teamcode.GaliV3.v3Hardware.shoulderPortScore;
import static org.firstinspires.ftc.teamcode.GaliV3.v3Hardware.shoulderStarScore;
import static org.firstinspires.ftc.teamcode.GaliV3.v3Roadrunner.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.GaliV3.v3Roadrunner.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.GaliV3.v3Roadrunner.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.GaliV3.v3Roadrunner.drive.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.GaliV3.v3Roadrunner.drive.SampleMecanumDrive.getAccelerationConstraint;
import static org.firstinspires.ftc.teamcode.GaliV3.v3Roadrunner.drive.SampleMecanumDrive.getVelocityConstraint;
import static org.firstinspires.ftc.teamcode.Vision.BlueColorProcessor.centerBlueRatio;
import static org.firstinspires.ftc.teamcode.Vision.BlueColorProcessor.leftBlueRatio;
import static org.firstinspires.ftc.teamcode.Vision.BlueColorProcessor.pos;
import static org.firstinspires.ftc.teamcode.Vision.BlueColorProcessor.rightBlueRatio;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.GaliV3.v3Hardware;
import org.firstinspires.ftc.teamcode.GaliV3.v3Roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.GaliV3.v3Roadrunner.trajectorysequence.TrajectorySequence;

import java.util.Objects;
@Autonomous
public class v3OnTapeScoreFareBlue extends v3autoBase{
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

        robot.door.setPosition(v3Hardware.doorClosed);
        TrajectorySequence center1 = drive.trajectorySequenceBuilder(startPose)
                .splineToLinearHeading(new Pose2d(-38, 10), Math.toRadians(0))
                .lineToLinearHeading(new Pose2d(-39, 10, Math.toRadians(-85)))
                .build();
        TrajectorySequence center2 = drive.trajectorySequenceBuilder(center1.end())
                .lineToLinearHeading(new Pose2d(-51, 14, Math.toRadians(-85)))
                .lineToLinearHeading(new Pose2d(-52, 14, Math.toRadians(85)))
                .lineToLinearHeading(new Pose2d(-52, -50, Math.toRadians(95)))
                .lineToLinearHeading(new Pose2d(-50, -80, Math.toRadians(95)))
                .lineToLinearHeading(new Pose2d(-43, -87, Math.toRadians(97)))
                .build();
        TrajectorySequence center3 = drive.trajectorySequenceBuilder(center2.end())
                .lineToLinearHeading(new Pose2d(-43, -91, Math.toRadians(90)),getVelocityConstraint(MAX_VEL/4, MAX_ANG_VEL/4, TRACK_WIDTH),getAccelerationConstraint(MAX_ACCEL/2))
                .build();
        TrajectorySequence left1 = drive.trajectorySequenceBuilder(startPose)
                .splineToLinearHeading(new Pose2d(-15, 9), Math.toRadians(0),getVelocityConstraint(MAX_VEL/3, MAX_ANG_VEL/3, TRACK_WIDTH),getAccelerationConstraint(MAX_ACCEL/2))
                .lineToLinearHeading(new Pose2d(-16, 9, Math.toRadians(-175)),getVelocityConstraint(MAX_VEL/3, MAX_ANG_VEL/3, TRACK_WIDTH),getAccelerationConstraint(MAX_ACCEL/2))
                .build();
        TrajectorySequence left2 = drive.trajectorySequenceBuilder(left1.end())
                .lineToLinearHeading(new Pose2d(-10, 9, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(-10, -2, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(-50, -2, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(-50, 5, Math.toRadians(110)))
                .lineToLinearHeading(new Pose2d(-53, 5, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(-53, -50, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(-53, -86, Math.toRadians(85)))
                .build();
        TrajectorySequence left3 = drive.trajectorySequenceBuilder(left2.end())
                .lineToLinearHeading(new Pose2d(-50, -90, Math.toRadians(80)))
                .build();
        TrajectorySequence right1 = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-27, 4, Math.toRadians(-90)),getVelocityConstraint(MAX_VEL/3, MAX_ANG_VEL/4, TRACK_WIDTH),getAccelerationConstraint(MAX_ACCEL/2))
                .lineToLinearHeading(new Pose2d(-27, -14, Math.toRadians(-90)),getVelocityConstraint(MAX_VEL/3, MAX_ANG_VEL/4, TRACK_WIDTH),getAccelerationConstraint(MAX_ACCEL/2))
                .lineToLinearHeading(new Pose2d(-27, -5, Math.toRadians(-90)),getVelocityConstraint(MAX_VEL/3, MAX_ANG_VEL/4, TRACK_WIDTH),getAccelerationConstraint(MAX_ACCEL/2))
                .build();

        TrajectorySequence right2 = drive.trajectorySequenceBuilder(right1.end())
                .lineToLinearHeading(new Pose2d(-27, 0, Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(-27, 1, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(-50, 1, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(-52, 1, Math.toRadians(93)))
                .lineToLinearHeading(new Pose2d(-52, -20, Math.toRadians(93)))
                .lineToLinearHeading(new Pose2d(-51, -80, Math.toRadians(93)))
                .lineToLinearHeading(new Pose2d(-29, -86, Math.toRadians(93)))
                .build();
        TrajectorySequence right3 = drive.trajectorySequenceBuilder(right2.end())
                .lineToLinearHeading(new Pose2d(-29, -90, Math.toRadians(95)),getVelocityConstraint(MAX_VEL/3, MAX_ANG_VEL/4, TRACK_WIDTH),getAccelerationConstraint(MAX_ACCEL/2))
                .build();
        while (!isStarted()) {
            telemetry.addData("position", propPos("blue", "far"));
            telemetry.addData("rightBlue", rightBlueRatio);
            telemetry.addData("centerBlue", centerBlueRatio);
            telemetry.addData("leftBlue", leftBlueRatio);
            telemetry.update();
        }
        telemetry.addData("position", propPos("blue", "far"));
        telemetry.addData("rightBlue", rightBlueRatio);
        telemetry.addData("centerBlue", centerBlueRatio);
        telemetry.addData("leftBlue", leftBlueRatio);
        telemetry.update();
        waitForStart();
        if (propPos.equals("center")) {
            drive.followTrajectorySequence(center1);
            robot.intake.setPower(-0.5);
            sleep(800);
            robot.intake.setPower(0);
            sleep(1000);
            drive.followTrajectorySequence(center2);
            resetRuntime();
            robot.portArm.setPower(0.15);
            robot.starArm.setPower(0.05);
            scoreBack();
            sleep(2000);
            scoreStar();
            sleep(1000);
            robot.starArm.setPower(0);
            robot.portArm.setPower(0);
            drive.followTrajectorySequence(center3);
            sleep(1000);
            robot.door.setPosition(v3Hardware.doorOpen);
            resetRuntime();
            while (getRuntime() < 0.3) {
                robot.portArm.setPower(1);
                robot.starArm.setPower(1);
            }
            robot.portArm.setPower(0);
            robot.starArm.setPower(0);
            robot.shoulderStar.setPosition(shoulderStarScore - 0.04);
            robot.shoulderPort.setPosition(shoulderPortScore + 0.04);
            robot.elbow.setPosition(elbowNorminal);
        } else if (propPos.equals("right")) {
            drive.followTrajectorySequence(left1);
            robot.intake.setPower(-0.5);
            sleep(800);
            robot.intake.setPower(0);
            sleep(1000);
            drive.followTrajectorySequence(left2);
            resetRuntime();
            robot.portArm.setPower(0.15);
            robot.starArm.setPower(0.05);
            scoreBack();
            sleep(2000);
            scoreStar();
            sleep(1000);
            robot.starArm.setPower(0);
            robot.portArm.setPower(0);
            drive.followTrajectorySequence(left3);
            sleep(1000);
            robot.door.setPosition(v3Hardware.doorOpen);
            resetRuntime();
            while (getRuntime() < 0.3) {
                robot.portArm.setPower(1);
                robot.starArm.setPower(1);
            }
            robot.portArm.setPower(0);
            robot.starArm.setPower(0);
            robot.shoulderStar.setPosition(shoulderStarScore - 0.04);
            robot.shoulderPort.setPosition(shoulderPortScore + 0.04);
            robot.elbow.setPosition(elbowNorminal);
            } else {
            drive.followTrajectorySequence(right1);
            robot.intake.setPower(-0.5);
            sleep(800);
            robot.intake.setPower(0);
            sleep(1000);
            drive.followTrajectorySequence(right2);
            resetRuntime();
            robot.portArm.setPower(0.15);
            robot.starArm.setPower(0.05);
            scoreBack();
            sleep(2000);
            scoreStar();
            sleep(1000);
            robot.starArm.setPower(0);
            robot.portArm.setPower(0);
            drive.followTrajectorySequence(right3);
            sleep(1000);
            robot.door.setPosition(v3Hardware.doorOpen);
            resetRuntime();
            while (getRuntime() < 0.3) {
                robot.portArm.setPower(1);
                robot.starArm.setPower(1);
            }
            robot.portArm.setPower(0);
            robot.starArm.setPower(0);
            robot.shoulderStar.setPosition(shoulderStarScore - 0.04);
            robot.shoulderPort.setPosition(shoulderPortScore + 0.04);
            robot.elbow.setPosition(elbowNorminal);
        }
        resetArm();
    }
}
