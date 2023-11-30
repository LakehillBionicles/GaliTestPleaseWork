package org.firstinspires.ftc.teamcode.GaliAuto;

import static org.firstinspires.ftc.teamcode.GaliHardware.elbowDown;
import static org.firstinspires.ftc.teamcode.GaliHardware.elbowScore;
import static org.firstinspires.ftc.teamcode.GaliHardware.fingerPortClosed;
import static org.firstinspires.ftc.teamcode.GaliHardware.fingerPortOpen;
import static org.firstinspires.ftc.teamcode.GaliHardware.fingerStarClosed;
import static org.firstinspires.ftc.teamcode.GaliHardware.fingerStarOpen;
import static org.firstinspires.ftc.teamcode.GaliHardware.wristDown;
import static org.firstinspires.ftc.teamcode.GaliHardware.wristScore;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Roadrunner.trajectorysequence.TrajectorySequence;

@Config
@Autonomous
public class ScoreCloseBlue extends GaliAutobase {

    Pose2d startPose = new Pose2d(0, 0, 0);
    @Override
    public void runOpMode(){
        robot.init(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        TrajectorySequence park = drive.trajectorySequenceBuilder(startPose)
                .forward(30)
                .turn(Math.toRadians(88))
                .forward(38)
                .turn(Math.toRadians(205))
                .back(6)
                .build();

        robot.fingerPort.setPosition(fingerPortClosed);
        robot.fingerStar.setPosition(fingerStarClosed);

        waitForStart();

        drive.followTrajectorySequence(park);
        robot.wrist.setPosition(.6);
        robot.elbow.setPosition(elbowScore);
        robot.wrist.setPosition(wristScore);
        sleep(2000);
        robot.fingerPort.setPosition(fingerPortOpen);
        robot.fingerStar.setPosition(fingerStarOpen);
        sleep(3000);
        robot.wrist.setPosition(.6);
        robot.elbow.setPosition(elbowDown);
        robot.wrist.setPosition(wristDown);
        sleep(3000);
    }
}
