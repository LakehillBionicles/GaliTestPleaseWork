package org.firstinspires.ftc.teamcode.GaliV3.v3Auto;

import static org.firstinspires.ftc.teamcode.GaliV3.v3Hardware.elbowNorminal;
import static org.firstinspires.ftc.teamcode.GaliV3.v3Hardware.shoulderPortScore;
import static org.firstinspires.ftc.teamcode.GaliV3.v3Hardware.shoulderStarScore;
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
public class v3ScoreCloseBlue extends v3autoBase{
    Pose2d startPose = new Pose2d(0, 0, 0);
    @Override
    public void runOpMode() {
        super.runOpMode();
        double armTime = 0;
        double armTime2 = 0;
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        cameraStartup("Webcam 1");
        propDetection("blue");

        TrajectorySequence center1 = drive.trajectorySequenceBuilder(startPose)
                .splineToLinearHeading(new Pose2d(-37, -12), Math.toRadians(0))
                .lineToLinearHeading(new Pose2d(-38, -12, Math.toRadians(85)))
                .build();

        TrajectorySequence center2 = drive.trajectorySequenceBuilder(center1.end())
                //.lineToLinearHeading(new Pose2d(-38.5, -18, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(-8, -36, Math.toRadians(90)))
                .build();
        TrajectorySequence center3 = drive.trajectorySequenceBuilder(center2.end())
                .lineToLinearHeading(new Pose2d(-8, -37, Math.toRadians(95)))
                .build();
        TrajectorySequence center4 = drive.trajectorySequenceBuilder(center3.end())
                .lineToLinearHeading(new Pose2d(0, -36, Math.toRadians(90)))
                .build();
        TrajectorySequence right1 = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-27, -4, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(-27, 10, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(-27, 6, Math.toRadians(90)))
                .build();

        TrajectorySequence right2 = drive.trajectorySequenceBuilder(right1.end())
                .lineToLinearHeading(new Pose2d(-20, -36, Math.toRadians(90)))
                .build();
        TrajectorySequence right3 = drive.trajectorySequenceBuilder(right2.end())
                .lineToLinearHeading(new Pose2d(-20, -37, Math.toRadians(95)))
                .build();
        TrajectorySequence right4 = drive.trajectorySequenceBuilder(right3.end())
                .lineToLinearHeading(new Pose2d(0, -36, Math.toRadians(90)))
                .build();

        TrajectorySequence left1 = drive.trajectorySequenceBuilder(startPose)
                .splineToLinearHeading(new Pose2d(-15, -9), Math.toRadians(0))
                .lineToLinearHeading(new Pose2d(-16, -9, Math.toRadians(165)))
                .build();

        TrajectorySequence left2 = drive.trajectorySequenceBuilder(left1.end())
                .lineToLinearHeading(new Pose2d(-13, -9, Math.toRadians(165)))
                .lineToLinearHeading(new Pose2d(-5, -36, Math.toRadians(90)))
                .build();
        TrajectorySequence left3 = drive.trajectorySequenceBuilder(left2.end())
                .lineToLinearHeading(new Pose2d(-5, -37, Math.toRadians(95)))
                .build();
        TrajectorySequence left4 = drive.trajectorySequenceBuilder(left3.end())
                .lineToLinearHeading(new Pose2d(0, -36, Math.toRadians(90)))
                .build();
        while (!opModeIsActive()) {
            telemetry.addData("position", pos);
            telemetry.addData("leftBlue", leftBlueRatio);
            telemetry.addData("centerBlue", centerBlueRatio);
            telemetry.addData("rightBlue", rightBlueRatio);
            propPos = pos;
            telemetry.update();
        }
        waitForStart();
        if(propPos.equals("center")) {
            drive.followTrajectorySequence(center1);
            robot.intake.setPower(-0.5);
            sleep(200);
            scoreBack();
            sleep(600);
            robot.intake.setPower(0);
            sleep(150);
            scorePort();
            robot.portArm.setPower(0.15);
            robot.starArm.setPower(0.05);
            drive.followTrajectorySequence(center2);
            robot.starArm.setPower(0);
            robot.portArm.setPower(0);
            drive.followTrajectorySequence(center3);
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
            drive.followTrajectorySequence(center4);
        }
        else if(propPos.equals("left")){
        drive.followTrajectorySequence(left1);
        robot.intake.setPower(-0.5);
        sleep(200);
        scoreBack();
        sleep(600);
        robot.intake.setPower(0);
        sleep(150);
        scorePort();
        robot.portArm.setPower(0.15);
        robot.starArm.setPower(0.07);
        drive.followTrajectorySequence(left2);
        robot.starArm.setPower(0);
        robot.portArm.setPower(0);
        drive.followTrajectorySequence(left3);
        robot.door.setPosition(v3Hardware.doorOpen);
        armTime2 = getRuntime();
        while (armTime < 0.3) {
            armTime = getRuntime() - armTime2;
            robot.portArm.setPower(1);
            robot.starArm.setPower(1);

        }
            robot.portArm.setPower(0);
            robot.starArm.setPower(0);
            robot.shoulderStar.setPosition(shoulderStarScore - 0.04);
            robot.shoulderPort.setPosition(shoulderPortScore + 0.04);
            robot.elbow.setPosition(elbowNorminal);
            telemetry.addData("Runtime", getRuntime()+1);
            telemetry.update();
            drive.followTrajectorySequence(left4);
}
        else{
        drive.followTrajectorySequence(right1);
        robot.intake.setPower(-0.5);
        sleep(200);
        scoreBack();
        sleep(600);
        robot.intake.setPower(0);
        sleep(150);
        scorePort();
        robot.portArm.setPower(0.15);
        robot.starArm.setPower(0.07);
        drive.followTrajectorySequence(right2);
        robot.starArm.setPower(0);
        robot.portArm.setPower(0);
        drive.followTrajectorySequence(right3);
        robot.door.setPosition(v3Hardware.doorOpen);
        armTime2 = getRuntime();
        while (armTime < 0.3) {
            armTime = getRuntime() - armTime2;
            robot.portArm.setPower(1);
            robot.starArm.setPower(1);

        }
        robot.portArm.setPower(0);
        robot.starArm.setPower(0);
        robot.shoulderStar.setPosition(shoulderStarScore - 0.04);
        robot.shoulderPort.setPosition(shoulderPortScore + 0.04);
        robot.elbow.setPosition(elbowNorminal);
        drive.followTrajectorySequence(right4);
    }
        resetArm();
    }
}
