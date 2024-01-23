package org.firstinspires.ftc.teamcode.GaliV3.v3Auto;


import static org.firstinspires.ftc.teamcode.Vision.BlueColorProcessor.centerBlueRatio;
import static org.firstinspires.ftc.teamcode.Vision.BlueColorProcessor.leftBlueRatio;
import static org.firstinspires.ftc.teamcode.Vision.BlueColorProcessor.pos;
import static org.firstinspires.ftc.teamcode.Vision.BlueColorProcessor.rightBlueRatio;
import static org.firstinspires.ftc.teamcode.Vision.RedColorProcessor.rightRedRatio;
import static org.firstinspires.ftc.teamcode.Vision.RedColorProcessor.centerRedRatio;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.GaliV3.v3Hardware;
import org.firstinspires.ftc.teamcode.GaliV3.v3Roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.GaliV3.v3Roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Vision.RedColorProcessor;

import java.util.Objects;

@Autonomous
public class v3ScoreCloseRed extends v3autoBase{
    Pose2d startPose = new Pose2d(0, 0, 0);
    @Override
    public void runOpMode() {
        super.runOpMode();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        robot.door.setPosition(v3Hardware.doorClosed);

        cameraStartup("Webcam 1");
        propDetection("red");

        TrajectorySequence center1 = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(-39, 2.5))
                .turn(Math.toRadians(-110))
                .build();

        TrajectorySequence center2 = drive.trajectorySequenceBuilder(center1.end())
                .turn(Math.toRadians(20))
                .lineTo(new Vector2d(-42., 33))
                .build();
        TrajectorySequence center3 = drive.trajectorySequenceBuilder(center2.end())
                .lineTo((new Vector2d(-10, 32)))
                .build();

        TrajectorySequence left1 = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(-29, 0))
                .turn(Math.toRadians(105))
                .lineToLinearHeading(new Pose2d(-27, -20, Math.toRadians(100)))
                .lineToLinearHeading(new Pose2d(-27, -2, Math.toRadians(92)))
                .build();

        TrajectorySequence left2 = drive.trajectorySequenceBuilder(left1.end())
                .lineToLinearHeading(new Pose2d(-27, 5, Math.toRadians(92)))
                .turn(Math.toRadians(-200))
                .lineToLinearHeading(new Pose2d(-36, 35, Math.toRadians(-94)))
                .build();

        TrajectorySequence right1 = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(-35, 7.5))
                .lineTo(new Vector2d(-15, 7.5))
                .build();

        TrajectorySequence right2 = drive.trajectorySequenceBuilder(right1.end())
                .lineTo(new Vector2d(-10, 7.5))
                .turn(Math.toRadians(-107))
                .lineToLinearHeading(new Pose2d(-20, 33, Math.toRadians(-98)))
                .build();

        while(!opModeIsActive()) {
            telemetry.addData("position", org.firstinspires.ftc.teamcode.Vision.RedColorProcessor.pos);
            telemetry.addData("centerBlue", centerRedRatio);
            telemetry.addData("rightBlue", rightRedRatio);
            propPos = org.firstinspires.ftc.teamcode.Vision.RedColorProcessor.pos;
            telemetry.update();
        }
        waitForStart();
        /*
        drive.followTrajectorySequence(right1);
        robot.wrist.setPosition(wristTape);
        robot.elbow.setPosition(elbowTape);
        sleep(3000);
        robot.fingerPort.setPosition(fingerPortOpen);
        sleep(3000);
        robot.wrist.setPosition(wristScore);
        robot.elbow.setPosition(elbowScore);
        sleep(1000);
        drive.followTrajectorySequence(right2);
        sleep(3000);
        robot.fingerStar.setPosition(fingerStarOpen);
        sleep(2000);
        robot.elbow.setPosition(elbowDown);
        robot.wrist.setPosition(wristDown);
        sleep(3000);
        */
       // if(Objects.equals(propPos, "center")){
            drive.followTrajectorySequence(center1);
            sleep(1000);
            robot.intake.setPower(-0.5);
            sleep(800);
            robot.intake.setPower(0);
            scoreBack();
            sleep(1000);
            drive.followTrajectorySequence(center2);
            sleep(3000);
            robot.door.setPosition(v3Hardware.doorOpen);
            sleep(2000);
            resetArm();
            drive.followTrajectorySequence(center3);
        //}
        /*else if(Objects.equals(propPos, "left")){
            drive.followTrajectorySequence(left1);
            sleep(1000);
            robot.intake.setPower(-0.5);
            sleep(2000);
            robot.intake.setPower(0);
            scoreBack();
            sleep(1000);
            drive.followTrajectorySequence(left2);
            sleep(3000);
            robot.door.setPosition(v3Hardware.doorOpen);
            sleep(2000);
            resetArm();
        } else {
            drive.followTrajectorySequence(right1);
            sleep(1000);
            robot.intake.setPower(-0.5);
            sleep(2000);
            robot.intake.setPower(0);
            scoreBack();
            sleep(1000);
            drive.followTrajectorySequence(right2);
            sleep(3000);
            robot.door.setPosition(v3Hardware.doorOpen);
            sleep(2000);
            resetArm();
        }

         */
    }
}
