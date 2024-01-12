package org.firstinspires.ftc.teamcode.GaliV3.v3Auto;
import static org.firstinspires.ftc.teamcode.GaliHardware.elbowDown;
import static org.firstinspires.ftc.teamcode.GaliHardware.elbowScore;
import static org.firstinspires.ftc.teamcode.GaliHardware.elbowTape;
import static org.firstinspires.ftc.teamcode.GaliHardware.fingerPortClosed;
import static org.firstinspires.ftc.teamcode.GaliHardware.fingerPortOpen;
import static org.firstinspires.ftc.teamcode.GaliHardware.fingerStarClosed;
import static org.firstinspires.ftc.teamcode.GaliHardware.fingerStarOpen;
import static org.firstinspires.ftc.teamcode.GaliHardware.wristDown;
import static org.firstinspires.ftc.teamcode.GaliHardware.wristScore;
import static org.firstinspires.ftc.teamcode.GaliHardware.wristTape;
import static org.firstinspires.ftc.teamcode.Vision.BlueColorProcessor.centerBlueRatio;
import static org.firstinspires.ftc.teamcode.Vision.BlueColorProcessor.leftBlueRatio;
import static org.firstinspires.ftc.teamcode.Vision.BlueColorProcessor.pos;
import static org.firstinspires.ftc.teamcode.Vision.RedColorProcessor.centerRedRatio;
import static org.firstinspires.ftc.teamcode.Vision.RedColorProcessor.rightRedRatio;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.GaliV3.v3Hardware;
import org.firstinspires.ftc.teamcode.GaliV3.v3Roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.GaliV3.v3Roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Vision.RedColorProcessor;

import java.util.Objects;
public class onTakeScoreFareRed extends v3autoBase{
    Pose2d startPose = new Pose2d(0, 0, 0);
    public static double forWard = 0;
    public static double turn1 = 0;
    @Override
    public void runOpMode() {
        super.runOpMode();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        cameraStartup("Webcam 1");
        propDetection("red");

        TrajectorySequence center1 = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-27, -15, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(-40, -15, Math.toRadians(182)))
                .lineToLinearHeading(new Pose2d(-40, -3, Math.toRadians(182)))
                .lineToLinearHeading(new Pose2d(-52, -3, Math.toRadians(182)))
                .build();
        TrajectorySequence center2 = drive.trajectorySequenceBuilder(center1.end())
                .lineToLinearHeading(new Pose2d(-60, -3, Math.toRadians(274)))
                .lineToLinearHeading(new Pose2d(-60, 70, Math.toRadians(274)))
                .lineToLinearHeading(new Pose2d(-34, 86, Math.toRadians(274)))
                .build();

        TrajectorySequence left1 = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(-29, 0))
                .turn(Math.toRadians(105))
                .lineToLinearHeading(new Pose2d(-27, -15, Math.toRadians(100)))
                .lineToLinearHeading(new Pose2d(-27, 3, Math.toRadians(92)))
                .build();
        TrajectorySequence left2 = drive.trajectorySequenceBuilder(left1.end())
                .lineToLinearHeading(new Pose2d(-27, 6, Math.toRadians(92)))
                .lineToLinearHeading(new Pose2d(-27, 5, Math.toRadians(-92)))
                .lineToLinearHeading(new Pose2d(-55, 5, Math.toRadians(-89)))
                .lineToLinearHeading(new Pose2d(-55, 70, Math.toRadians(-89)))
                .lineToLinearHeading(new Pose2d(-40, 87, Math.toRadians(-89)))
                .build();

        TrajectorySequence right1 = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-27, 0, Math.toRadians(0)))
                .turn(Math.toRadians(-105))
                .lineToLinearHeading(new Pose2d(-27, 15, Math.toRadians(-100)))
                .lineToLinearHeading(new Pose2d(-27, 2, Math.toRadians(-92)))
                .build();

        TrajectorySequence right2 = drive.trajectorySequenceBuilder(right1.end())
                .lineToLinearHeading(new Pose2d(-27, 0, Math.toRadians(-92)))
                .lineToLinearHeading(new Pose2d(-55, 0, Math.toRadians(-89)))
                .lineToLinearHeading(new Pose2d(-55, 70, Math.toRadians(-89)))
                .lineToLinearHeading(new Pose2d(-22, 86, Math.toRadians(-89)))
                .build();



        while(!isStarted()) {
            telemetry.addData("position", org.firstinspires.ftc.teamcode.Vision.RedColorProcessor.pos);
            telemetry.addData("centerBlue", centerRedRatio);
            telemetry.addData("rightBlue", rightRedRatio);
            telemetry.update();
        }
        telemetry.addData("position", org.firstinspires.ftc.teamcode.Vision.RedColorProcessor.pos);
        telemetry.addData("centerBlue", centerRedRatio);
        telemetry.addData("rightBlue", rightRedRatio);
        telemetry.update();
        waitForStart();
        if(Objects.equals(org.firstinspires.ftc.teamcode.Vision.RedColorProcessor.pos, "center")) {
            drive.followTrajectorySequence(center1);
            sleep(1000);
            robot.intake.setPower(-0.5);
            sleep(2000);
            robot.intake.setPower(0);
            drive.followTrajectorySequence(center2);
            scoreBack();
            sleep(3000);
            robot.door.setPosition(v3Hardware.doorOpen);
            sleep(2000);
            resetArm();
        }else if(Objects.equals(org.firstinspires.ftc.teamcode.Vision.RedColorProcessor.pos, "left")){
            drive.followTrajectorySequence(left1);
            sleep(1000);
            robot.intake.setPower(-0.5);
            sleep(2000);
            robot.intake.setPower(0);
            drive.followTrajectorySequence(left2);
            scoreBack();
            sleep(3000);
            robot.door.setPosition(v3Hardware.doorOpen);
            sleep(2000);
            resetArm();

        }else{
            drive.followTrajectorySequence(right1);
            sleep(1000);
            robot.intake.setPower(-0.5);
            sleep(2000);
            robot.intake.setPower(0);
            drive.followTrajectorySequence(right2);
            scoreBack();
            sleep(3000);
            robot.door.setPosition(v3Hardware.doorOpen);
            sleep(2000);
            resetArm();
        }
    }
}
