package org.firstinspires.ftc.teamcode.GaliV3.v3Roadrunner.Testing;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.GaliV3.v3Roadrunner.drive.SampleMecanumDrive;

@Disabled
@Config
@Autonomous
public class Back extends LinearOpMode {
    public TestHwMap robot = new TestHwMap();

    public double xTarget = -23.5;
    public double yTarget = 0;

    public void runOpMode(){
        robot.init(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
        drive.setPoseEstimate(startPose);

        Trajectory ToTape = drive.trajectoryBuilder(startPose)
                .lineToConstantHeading(new Vector2d(xTarget, yTarget))
                .build();

        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectory(ToTape);
    }

}
