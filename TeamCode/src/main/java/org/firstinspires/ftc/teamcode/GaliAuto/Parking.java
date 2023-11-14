package org.firstinspires.ftc.teamcode.GaliAuto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Roadrunner.trajectorysequence.TrajectorySequence;

@Config
@Autonomous
public class Parking extends GaliAutobase{

    Pose2d startPose = new Pose2d(0, 0, 0);
    @Override
    public void runOpMode(){
        robot.init(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        TrajectorySequence park = drive.trajectorySequenceBuilder(startPose)
                .forward(45)
                .build();

        waitForStart();

        drive.followTrajectorySequence(park);
    }
}
