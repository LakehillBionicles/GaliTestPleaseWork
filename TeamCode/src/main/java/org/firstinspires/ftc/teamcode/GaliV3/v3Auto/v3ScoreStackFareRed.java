package org.firstinspires.ftc.teamcode.GaliV3.v3Auto;

import static org.firstinspires.ftc.teamcode.Vision.BlueColorProcessor.centerBlueRatio;
import static org.firstinspires.ftc.teamcode.Vision.BlueColorProcessor.leftBlueRatio;
import static org.firstinspires.ftc.teamcode.Vision.BlueColorProcessor.rightBlueRatio;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.GaliV3.v3Hardware;
import org.firstinspires.ftc.teamcode.GaliV3.v3Roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.GaliV3.v3Roadrunner.trajectorysequence.TrajectorySequence;
@Autonomous
public class v3ScoreStackFareRed extends v3autoBase{
    Pose2d startPose = new Pose2d(0, 0, 0);
    public static double forWard = 0;
    public static double turn1 = 0;
    double armTime = 0;
    double armTime2 = 0;
    String distanceString = "not seen";
    double distanceTimer = -3;
    @Override
    public void runOpMode() {
        super.runOpMode();
        robot.intake.setPower(0.02);
        cameraStartup("Webcam 1");
        propDetection("red");
        propPos("red", "far");
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        robot.flipper.setPosition(v3Hardware.flipUp);
        robot.intake.setPower(0);
        robot.door.setPosition(v3Hardware.doorClosed);
        TrajectorySequence center1 = drive.trajectorySequenceBuilder(startPose)
                .back(5)
                .splineToLinearHeading((new Pose2d(-10, -10, Math.toRadians(0))),Math.toRadians(180))
                .splineToLinearHeading((new Pose2d(-50, -17, Math.toRadians(0))),Math.toRadians(180))
                .splineToLinearHeading((new Pose2d(-52.5, 5, Math.toRadians(0))),Math.toRadians(180))
                .splineToLinearHeading((new Pose2d(-35.5, 5, Math.toRadians(0))),Math.toRadians(90))
                .splineToLinearHeading((new Pose2d(-39.5,-10, Math.toRadians(0))),Math.toRadians(-90))
                .turn(Math.toRadians(90))
                .build();
        TrajectorySequence center2 = drive.trajectorySequenceBuilder(center1.end())
                .addDisplacementMarker(()->{
                    robot.intake.setPower(-v3Hardware.intakeSpeed/8);
                    sleep(1500);
                    robot.intake.setPower(0);
                })
                .back(2)
                .turn(Math.toRadians(180))
                .splineToLinearHeading((startPose).plus(new Pose2d(-48.5, -10, Math.toRadians(-90))),Math.toRadians(180))
                .splineToLinearHeading((startPose).plus(new Pose2d(-48.5, -12, Math.toRadians(-90))),Math.toRadians(-90))
                .build();
        TrajectorySequence center3 = drive.trajectorySequenceBuilder(center2.end())
                .forward(9)
                .build();
        TrajectorySequence center4 = drive.trajectorySequenceBuilder(center3.end())
                .back(5)
                .addDisplacementMarker(()->{
                    robot.flipper.setPosition(v3Hardware.flipDown);
                    sleep(200);
                    robot.intake.setPower(-v3Hardware.intakeSpeed);
                    sleep(1000);
                    robot.intake.setPower(0);
                })
                .waitSeconds(3)
                .splineToLinearHeading((startPose).plus(new Pose2d(-58.5, 5, Math.toRadians(-87))),Math.toRadians(90))
                .splineToLinearHeading((startPose).plus(new Pose2d(-58.5, 80, Math.toRadians(-87))),Math.toRadians(90))
                .splineToLinearHeading((startPose).plus(new Pose2d(-32.5, 80, Math.toRadians(-87))),Math.toRadians(0))
                .splineToLinearHeading((startPose).plus(new Pose2d(-32.5, 97, Math.toRadians(-87))),Math.toRadians(90))
                .build();
        TrajectorySequence right1 = drive.trajectorySequenceBuilder(startPose)
                .back(20)
                .turn(Math.toRadians(90))
                .splineToLinearHeading((startPose).plus(new Pose2d(-28, -15, Math.toRadians(90))),Math.toRadians(180))
                .forward(16)
                .build();
        TrajectorySequence right2 = drive.trajectorySequenceBuilder(right1.end())
                .addDisplacementMarker(()->{
                    robot.flipper.setPosition(v3Hardware.flipDown);
                    robot.intake.setPower(-v3Hardware.intakeSpeed);
                    sleep(1000);
                    robot.intake.setPower(0);
                    robot.flipper.setPosition(v3Hardware.flipUp);
                })
                .back(5)
                .turn(Math.toRadians(180))
                .splineToLinearHeading((startPose).plus(new Pose2d(-50, -16, Math.toRadians(-90))),Math.toRadians(-90))
                .build();
        TrajectorySequence right3= drive.trajectorySequenceBuilder(right2.end())
                .forward(5.5)
                .build();
        TrajectorySequence right4= drive.trajectorySequenceBuilder(right3.end())
                .back(5)
                .addDisplacementMarker(()->{
                    robot.intake.setPower(v3Hardware.intakeSpeed);
                    robot.flipper.setPosition(v3Hardware.flipDown);
                    sleep(1000);
                    robot.intake.setPower(-v3Hardware.intakeSpeed);
                    sleep(1000);
                    robot.intake.setPower(0);
                })
                .splineToLinearHeading((startPose).plus(new Pose2d(-52, 86, Math.toRadians(-90))),Math.toRadians(90))
                .splineToLinearHeading((startPose).plus(new Pose2d(-26, 90, Math.toRadians(-90))),Math.toRadians(0))
                .splineToLinearHeading((startPose).plus(new Pose2d(-26, 93, Math.toRadians(-90))),Math.toRadians(90))
                .build();
        TrajectorySequence left1 = drive.trajectorySequenceBuilder(startPose)
                .back(5)
                .splineToLinearHeading((startPose).plus(new Pose2d(-10, 4, Math.toRadians(0))),Math.toRadians(180))
                .splineToLinearHeading((startPose).plus(new Pose2d(-40, 4, Math.toRadians(0))),Math.toRadians(180))
                .splineToLinearHeading((startPose).plus(new Pose2d(-46, -6, Math.toRadians(0))),Math.toRadians(-90))
                .splineToLinearHeading((startPose).plus(new Pose2d(-20, -7, Math.toRadians(0))),Math.toRadians(0))
                .back(23)
                .build();
        TrajectorySequence left2 = drive.trajectorySequenceBuilder(left1.end())
                .back(4)
                .addDisplacementMarker(()->{
                    robot.intake.setPower(-v3Hardware.intakeSpeed/2);
                    sleep(1000);
                    robot.intake.setPower(0);
                })
                .back(1)
                .waitSeconds(3)
                .turn(Math.toRadians(-90))
                .splineToLinearHeading((startPose).plus(new Pose2d(-50, -16, Math.toRadians(-90))),Math.toRadians(-90))
                .build();
        TrajectorySequence left3 = drive.trajectorySequenceBuilder(left2.end())
                .forward(5.4)
                .build();
        TrajectorySequence left4 = drive.trajectorySequenceBuilder(left3.end())
                .back(5)
                .addDisplacementMarker(()->{
                    robot.flipper.setPosition(v3Hardware.flipDown);
                    sleep(500);
                    robot.intake.setPower(-v3Hardware.intakeSpeed);
                    sleep(1000);
                    robot.intake.setPower(0);
                })
                .waitSeconds(1)
                .splineToLinearHeading((startPose).plus(new Pose2d(-55, 65, Math.toRadians(-90))),Math.toRadians(90))
                .splineToLinearHeading((startPose).plus(new Pose2d(-40, 85, Math.toRadians(-90))),Math.toRadians(90))
                .splineToLinearHeading((startPose).plus(new Pose2d(-37, 95, Math.toRadians(-90))),Math.toRadians(90))
                .build();
        while (!isStarted()) {
            telemetry.addData("position", propPos("red", "far"));
            telemetry.addData("rightBlue", rightBlueRatio);
            telemetry.addData("centerBlue", centerBlueRatio);
            telemetry.addData("leftBlue", leftBlueRatio);
            telemetry.addData("distance",robot.distanceTop.getDistance(DistanceUnit.MM));
            telemetry.update();
        }
        waitForStart();
/*middle
        drive.followTrajectorySequence(center1);
        robot.intake.setPower(0);
        robot.flipper.setPosition(v3Hardware.flipDown);
        sleep(700);
        robot.intake.setPower(-0.2);
        sleep(200);
        robot.intake.setPower(0);
        robot.flipper.setPosition(v3Hardware.flipUp);
        drive.followTrajectorySequence(center2);
        robot.intake.setPower(v3Hardware.intakeSpeed);
        robot.flipper.setPosition(v3Hardware.flipUp-((v3Hardware.flipUp-v3Hardware.flipDown)/1.85));
        drive.followTrajectorySequence(center3);
        drive.turn(Math.toRadians(10));
        sleep(500);
        drive.turn(Math.toRadians(-20));
        sleep(500);
        drive.turn(Math.toRadians(10));
        robot.flipper.setPosition(v3Hardware.flipUp-((v3Hardware.flipUp-v3Hardware.flipDown)/1.85));
        robot.intake.setPower(-v3Hardware.intakeSpeed);
        sleep(600);
        robot.intake.setPower(0);
        drive.followTrajectorySequence(center4);
        robot.shoulderStar.setPosition(v3Hardware.shoulderStarScore);
        robot.shoulderPort.setPosition(v3Hardware.shoulderPortScore);
        robot.portArm.setPower(1);
        robot.starArm.setPower(1);
        sleep(200);
        robot.portArm.setPower(0);
        robot.starArm.setPower(0);
        sleep(2000);
        robot.door.setPosition(v3Hardware.doorOpen);
        sleep(100);
        robot.elbow.setPosition(v3Hardware.elbowNorminal);
        robot.shoulderStar.setPosition(v3Hardware.shoulderStarScore-0.05);
        robot.shoulderPort.setPosition(v3Hardware.shoulderPortScore+0.05);
        sleep(300);
        robot.portArm.setPower(-1);
        robot.starArm.setPower(-1);
        robot.shoulderStar.setPosition(v3Hardware.shoulderStarDown);
        robot.shoulderPort.setPosition(v3Hardware.shoulderPortDown);
        robot.door.setPosition(v3Hardware.doorClosed);
        sleep(2000);
 */     if(propPos.equals("right")) {
            drive.followTrajectorySequence(right1);
            robot.intake.setPower(0);
            robot.flipper.setPosition(v3Hardware.flipDown);
            sleep(700);
            robot.intake.setPower(-0.2);
            sleep(400);
            robot.intake.setPower(0);
            robot.flipper.setPosition(v3Hardware.flipUp);
            drive.followTrajectorySequence(right2);
            robot.intake.setPower(v3Hardware.intakeSpeed);
            robot.flipper.setPosition(v3Hardware.flipUp - ((v3Hardware.flipUp - v3Hardware.flipDown) / 1.85));
            drive.followTrajectorySequence(right3);
            drive.turn(Math.toRadians(10));
            sleep(500);
            drive.turn(Math.toRadians(-20));
            sleep(500);
            drive.turn(Math.toRadians(10));
            robot.flipper.setPosition(v3Hardware.flipUp - ((v3Hardware.flipUp - v3Hardware.flipDown) / 1.85));
            robot.intake.setPower(-v3Hardware.intakeSpeed);
            sleep(600);
            robot.intake.setPower(0);
            drive.followTrajectorySequence(right4);
            drive.turn(Math.toRadians(5));
            robot.shoulderStar.setPosition(v3Hardware.shoulderStarScore);
            robot.shoulderPort.setPosition(v3Hardware.shoulderPortScore);
            sleep(1500);
            robot.portArm.setPower(1);
            robot.starArm.setPower(1);
            sleep(400);
            robot.portArm.setPower(0);
            robot.starArm.setPower(0);
            sleep(2000);
            resetRuntime();
            while (getRuntime() < 3 && !robot.handTS.isPressed()) {
                drive.setMotorPowers(-0.2, -0.2, -0.2, -0.2);
            }
            drive.setMotorPowers(-0, -0, -0, -0);
            robot.door.setPosition(v3Hardware.doorOpen);
            sleep(100);
            robot.elbow.setPosition(v3Hardware.elbowNorminal);
            robot.shoulderStar.setPosition(v3Hardware.shoulderStarScore - 0.05);
            robot.shoulderPort.setPosition(v3Hardware.shoulderPortScore + 0.05);
            sleep(300);
            robot.portArm.setPower(-1);
            robot.starArm.setPower(-1);
            robot.shoulderStar.setPosition(v3Hardware.shoulderStarDown);
            robot.shoulderPort.setPosition(v3Hardware.shoulderPortDown);
            robot.door.setPosition(v3Hardware.doorClosed);
            sleep(2000);
        }
        else if(propPos.equals("center")){
            drive.followTrajectorySequence(center1);
            robot.intake.setPower(0);
            robot.flipper.setPosition(v3Hardware.flipDown);
            sleep(700);
            robot.intake.setPower(-0.2);
            sleep(400);
            robot.intake.setPower(0);
            robot.flipper.setPosition(v3Hardware.flipUp);
            drive.followTrajectorySequence(center2);
            robot.intake.setPower(v3Hardware.intakeSpeed);
            robot.flipper.setPosition(v3Hardware.flipUp-((v3Hardware.flipUp-v3Hardware.flipDown)/1.85));
            drive.followTrajectorySequence(center3);
            drive.turn(Math.toRadians(10));
            sleep(500);
            drive.turn(Math.toRadians(-20));
            sleep(500);
            drive.turn(Math.toRadians(10));
            robot.flipper.setPosition(v3Hardware.flipUp-((v3Hardware.flipUp-v3Hardware.flipDown)/1.85));
            robot.intake.setPower(-v3Hardware.intakeSpeed);
            sleep(600);
            robot.intake.setPower(0);
            drive.followTrajectorySequence(center4);
            drive.turn(Math.toRadians(5));
            robot.shoulderStar.setPosition(v3Hardware.shoulderStarScore);
            robot.shoulderPort.setPosition(v3Hardware.shoulderPortScore);
            robot.portArm.setPower(1);
            robot.starArm.setPower(1);
            sleep(400);
            robot.portArm.setPower(0);
            robot.starArm.setPower(0);
            sleep(2000);
            resetRuntime();
            while(getRuntime()<3&&!robot.handTS.isPressed()){
                drive.setMotorPowers(-0.2,-0.2,-0.2,-0.2);
            }
            drive.setMotorPowers(-0,-0,-0,-0);
            robot.door.setPosition(v3Hardware.doorOpen);
            sleep(100);
            robot.elbow.setPosition(v3Hardware.elbowNorminal);
            robot.shoulderStar.setPosition(v3Hardware.shoulderStarScore-0.05);
            robot.shoulderPort.setPosition(v3Hardware.shoulderPortScore+0.05);
            sleep(300);
            robot.portArm.setPower(-1);
            robot.starArm.setPower(-1);
            robot.shoulderStar.setPosition(v3Hardware.shoulderStarDown);
            robot.shoulderPort.setPosition(v3Hardware.shoulderPortDown);
            robot.door.setPosition(v3Hardware.doorClosed);
            sleep(2000);
        }
            else{
            drive.followTrajectorySequence(left1);
            robot.intake.setPower(0);
            robot.flipper.setPosition(v3Hardware.flipDown);
            sleep(700);
            robot.intake.setPower(-0.2);
            sleep(400);
            robot.intake.setPower(0);
            robot.flipper.setPosition(v3Hardware.flipUp);
            drive.followTrajectorySequence(left2);
            robot.intake.setPower(v3Hardware.intakeSpeed);
            robot.flipper.setPosition(v3Hardware.flipUp-((v3Hardware.flipUp-v3Hardware.flipDown)/1.85));
            drive.followTrajectorySequence(left3);
            drive.turn(Math.toRadians(10));
            sleep(500);
            drive.turn(Math.toRadians(-20));
            sleep(500);
            drive.turn(Math.toRadians(10));
            robot.flipper.setPosition(v3Hardware.flipUp-((v3Hardware.flipUp-v3Hardware.flipDown)/1.85));
            robot.intake.setPower(-v3Hardware.intakeSpeed);
            sleep(600);
            robot.intake.setPower(0);
            drive.followTrajectorySequence(left4);
            drive.turn(Math.toRadians(5));
            robot.shoulderStar.setPosition(v3Hardware.shoulderStarScore);
            robot.shoulderPort.setPosition(v3Hardware.shoulderPortScore);
            robot.portArm.setPower(1);
            robot.starArm.setPower(1);
            sleep(400);
            robot.portArm.setPower(0);
            robot.starArm.setPower(0);
            sleep(2000);
            resetRuntime();
            while(getRuntime()<3&&!robot.handTS.isPressed()){
                drive.setMotorPowers(-0.2,-0.2,-0.2,-0.2);
            }
            drive.setMotorPowers(-0,-0,-0,-0);
            robot.door.setPosition(v3Hardware.doorOpen);
            sleep(100);
            robot.elbow.setPosition(v3Hardware.elbowNorminal);
            robot.shoulderStar.setPosition(v3Hardware.shoulderStarScore-0.05);
            robot.shoulderPort.setPosition(v3Hardware.shoulderPortScore+0.05);
            sleep(300);
            robot.portArm.setPower(-1);
            robot.starArm.setPower(-1);
            robot.shoulderStar.setPosition(v3Hardware.shoulderStarDown);
            robot.shoulderPort.setPosition(v3Hardware.shoulderPortDown);
            robot.door.setPosition(v3Hardware.doorClosed);
            sleep(2000);
        }
    }
    //Fancy April Tag

}
