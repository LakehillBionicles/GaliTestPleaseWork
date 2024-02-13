package org.firstinspires.ftc.teamcode.GaliV3.v3Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
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
@Autonomous

public class meepmeepTest extends v3autoBase {
    Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
    public static double forWard = 0;
    public static double turn1 = 0;
    double armTime = 0;
    double armTime2 = 0;

    @Override
    public void runOpMode() {
        super.runOpMode();
        robot.flipper.setPosition(v3Hardware.flipUp);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        TrajectorySequence center1 = drive.trajectorySequenceBuilder(startPose)
                /*
                .back(10)
                .turn(Math.toRadians(180))
                .splineToLinearHeading((new Pose2d(-20, -9, Math.toRadians(180))), Math.toRadians(180))
                .waitSeconds(1)
                .splineToLinearHeading((new Pose2d(-10, -9, Math.toRadians(180))), Math.toRadians(0))
                .turn(Math.toRadians(-90))
                .splineToLinearHeading((new Pose2d(-3, -35, Math.toRadians(90))), Math.toRadians(-90))

                 */
                //Switch x and y
                //make y opposite sign
                //heading switches sign
                //Tangent -90
                .back(5)
                .splineToLinearHeading((new Pose2d(-15, -8, Math.toRadians(180))),Math.toRadians(180))
                .splineToLinearHeading((new Pose2d(-13, 4, Math.toRadians(180))),Math.toRadians(-180))
                .splineToLinearHeading((new Pose2d(-30, 4, Math.toRadians(180))),Math.toRadians(-180))
                .splineToLinearHeading((new Pose2d(-50, -16, Math.toRadians(-90))),Math.toRadians(90))
                .addDisplacementMarker(()->{
                robot.flipper.setPosition(v3Hardware.flipDown);
                sleep(500);
                robot.intake.setTargetPosition(-41);
                robot.intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.intake.setPower(0.1);
                sleep(500);
                })
                .waitSeconds(1)
                .back(20)
                .splineToLinearHeading((new Pose2d(-50, 65, Math.toRadians(90))),Math.toRadians(-90))
                .splineToLinearHeading((new Pose2d(-45, 85, Math.toRadians( 90))),Math.toRadians(-90))

                //blue far middle
                //.splineToLinearHeading((startPose).minus(new Pose2d(10, 37, Math.toRadians(90))),Math.toRadians(0))
                .build();
        waitForStart();
        if (opModeIsActive()) {
            // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
            drive.followTrajectorySequence(center1);
        }
    }
}
