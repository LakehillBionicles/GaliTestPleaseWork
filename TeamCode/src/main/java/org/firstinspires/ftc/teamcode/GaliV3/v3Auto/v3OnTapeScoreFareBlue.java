package org.firstinspires.ftc.teamcode.GaliV3.v3Auto;
import static org.firstinspires.ftc.teamcode.Vision.BlueColorProcessor.centerBlueRatio;
import static org.firstinspires.ftc.teamcode.Vision.BlueColorProcessor.leftBlueRatio;
import static org.firstinspires.ftc.teamcode.Vision.BlueColorProcessor.pos;
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
    @Override
    public void runOpMode() {
        super.runOpMode();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        cameraStartup("Webcam 1");
        propDetection("blue");
        TrajectorySequence center1 = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-27, 18, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(-40, 18, Math.toRadians(-182)))
                .lineToLinearHeading(new Pose2d(-40, 6, Math.toRadians(-182)))
                .lineToLinearHeading(new Pose2d(-52, 6, Math.toRadians(-182)))
                .build();
        TrajectorySequence center2 = drive.trajectorySequenceBuilder(center1.end())
                .lineToLinearHeading(new Pose2d(-60, 6, Math.toRadians(-274)))
                .lineToLinearHeading(new Pose2d(-60, -67, Math.toRadians(-274)))
                .lineToLinearHeading(new Pose2d(-34, -84, Math.toRadians(-274)))
                .build();

        TrajectorySequence left1 = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(-29, 0))
                .turn(Math.toRadians(-105))
                .lineToLinearHeading(new Pose2d(-27, 18, Math.toRadians(-100)))
                .lineToLinearHeading(new Pose2d(-27, 0, Math.toRadians(-92)))
                .build();
        TrajectorySequence left2 = drive.trajectorySequenceBuilder(left1.end())
                .lineToLinearHeading(new Pose2d(-27, -3, Math.toRadians(-92)))
                .lineToLinearHeading(new Pose2d(-27, -2, Math.toRadians(92)))
                .lineToLinearHeading(new Pose2d(-55, -2, Math.toRadians(89)))
                .lineToLinearHeading(new Pose2d(-55, -67, Math.toRadians(89)))
                .lineToLinearHeading(new Pose2d(-40, -83, Math.toRadians(89)))
                .build();

        TrajectorySequence right1 = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-27, 3, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(-27, 14, Math.toRadians(-100)))
                .lineToLinearHeading(new Pose2d(-29, 2.5, Math.toRadians(-100)))
                .build();

        TrajectorySequence right2 = drive.trajectorySequenceBuilder(right1.end())
                .lineToLinearHeading(new Pose2d(-29, 2, Math.toRadians(-100)))
                .lineToLinearHeading(new Pose2d(-57, 2, Math.toRadians(-100)))
                .lineToLinearHeading(new Pose2d(-57, -4, Math.toRadians(89)))
                .lineToLinearHeading(new Pose2d(-57, -67, Math.toRadians(89)))
                .lineToLinearHeading(new Pose2d(-34, -83, Math.toRadians(89)))
                .build();



        while(!isStarted()) {
            telemetry.addData("position", pos);
            telemetry.addData("centerBlue", centerBlueRatio);
            telemetry.addData("leftBlue", leftBlueRatio);
            telemetry.update();
        }
        telemetry.addData("position", pos);
        telemetry.addData("centerBlue", centerBlueRatio);
        telemetry.addData("leftBlue", leftBlueRatio);
        telemetry.update();
        waitForStart();
        if(Objects.equals(pos, "center")) {
            drive.followTrajectorySequence(center1);
            sleep(1000);
            robot.intake.setPower(-0.5);
            sleep(2000);
            robot.intake.setPower(0);
            drive.followTrajectorySequence(center2);
            scoreBack();
            sleep(1000);
            robot.door.setPosition(v3Hardware.doorOpen);
            sleep(2000);
            resetArm();
            sleep(3000);
        }else if(Objects.equals(pos, "left")){
            drive.followTrajectorySequence(left1);
            sleep(1000);
            robot.intake.setPower(-0.5);
            sleep(2000);
            robot.intake.setPower(0);
            drive.followTrajectorySequence(left2);
            sleep(1500);
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
            sleep(4000);
        }
    }
}
