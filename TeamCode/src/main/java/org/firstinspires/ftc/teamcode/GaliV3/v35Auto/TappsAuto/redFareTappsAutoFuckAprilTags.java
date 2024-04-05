package org.firstinspires.ftc.teamcode.GaliV3.v35Auto.TappsAuto;

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
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.GaliV3.v3Auto.v3autoBase;
import org.firstinspires.ftc.teamcode.GaliV3.v3Hardware;
import org.firstinspires.ftc.teamcode.GaliV3.v3Roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.GaliV3.v3Roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Vision.AprilTagDetectionPipeline2;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
@Autonomous

public class redFareTappsAutoFuckAprilTags extends v3autoBase {
    Pose2d startPose = new Pose2d(-38, 61, Math.toRadians(-90));
    public String  propPos = "notSeen";
    public static double forWard = 0;
    public static double turn1 = 0;
    double armTime = 0;
    double armTime2 = 0;
    String distanceString = "not seen";
    double distanceTimer = -3;
    double headingError = 0;
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;
    double tagsize = 0.05;
    public IMU imu;
    boolean stayInLoop =true;
    OpenCvCamera camera;
    AprilTagDetectionPipeline2 aprilTagDetectionPipeline2= new AprilTagDetectionPipeline2(tagsize, fx, fy, cx, cy);
    static final double FEET_PER_METER = 3.28084;
    @Override
    public void runOpMode() {
        super.runOpMode();
        imu = hardwareMap.get(IMU.class, "imu");
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        v3autoBase.pipeline = "propRed";
        v3autoBase.robotPosition = "far";
        /*
        cameraStartup("Webcam 1");
        propDetection("red");
        propPos("red", "far");
        while (!isStarted()&&!gamepad1.b) {
            telemetry.addData("position", propPos("red", "far"));
            telemetry.addData("rightBlue", rightBlueRatio);
            telemetry.addData("centerBlue", centerBlueRatio);
            telemetry.addData("leftBlue", leftBlueRatio);
            telemetry.update();
        }
        */int numFramesWithoutDetection = 0;

        final float DECIMATION_HIGH = 3;
        final float DECIMATION_LOW = 2;
        final float THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f;
        final int THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION = 4;
        IMU.Parameters myIMUParameters;
        myIMUParameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        new Orientation(
                                AxesReference.INTRINSIC,
                                AxesOrder.ZYX,
                                AngleUnit.DEGREES,
                                0,
                                90,
                                60,
                                0  // acquisitionTime, not used
                        )
                ));
        imu.initialize(myIMUParameters);
        imu.resetYaw();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline2 = new AprilTagDetectionPipeline2(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline2);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });
        robot.flipper.setPosition(v3Hardware.flipUp);
        robot.door.setPosition(v3Hardware.doorClosed);

        TrajectorySequence center1 = drive.trajectorySequenceBuilder(startPose)
                .back(9)
                .splineTo((new Vector2d(startPose.getX(),startPose.getY())).plus(new Vector2d(-2, 30.5)), Math.toRadians(90))
                //drop pixels
                .forward(10)
                .turn(Math.toRadians(-90))
                .addDisplacementMarker(()->{
                    robot.flipper.setPosition(v3Hardware.flipDown);
                })
                .waitSeconds(0.2)
                .splineToLinearHeading((startPose).plus(new Pose2d(-20, 13, Math.toRadians(-90))), Math.toRadians(180))
                .waitSeconds(0.2)
                .splineToLinearHeading((startPose).plus(new Pose2d(-20, 27.5, Math.toRadians(-90))), Math.toRadians(90))
                .build();
        TrajectorySequence center2 = drive.trajectorySequenceBuilder(center1.end())
                /*
                .back(0.1)
                .splineToLinearHeading((startPose).plus(new Pose2d(-10, 51.5, Math.toRadians(-90))), Math.toRadians(0))
                .splineToLinearHeading((startPose).plus(new Pose2d(0, 52.5, Math.toRadians(-90))),Math.toRadians(0))
                .splineToLinearHeading((startPose).plus(new Pose2d(68, 52.5, Math.toRadians(-90))),Math.toRadians(0)
                */
                .back(4)
                .waitSeconds(0.1)
                .strafeRight(24.5)
                .turn(Math.toRadians(4))
                .waitSeconds(0.1)
                .back(90)
                .splineToLinearHeading((startPose).plus(new Pose2d(78, 32, Math.toRadians(-90))),Math.toRadians(-90))
                .build();
        TrajectorySequence right1 = drive.trajectorySequenceBuilder(startPose)
                .back(1)
                //.waitSeconds(0.1)
                //.strafeRight(10)
                //.waitSeconds(0.1)
                //.back(2)
                .splineTo((new Vector2d(startPose.getX(),startPose.getY())).plus(new Vector2d(-10, 14)), Math.toRadians(90),getVelocityConstraint(MAX_VEL/1.2, MAX_ANG_VEL/2, TRACK_WIDTH),getAccelerationConstraint(MAX_ACCEL/4))
                .splineTo((new Vector2d(startPose.getX(),startPose.getY())).plus(new Vector2d(-3, 26)), Math.toRadians(0),getVelocityConstraint(MAX_VEL/1.2, MAX_ANG_VEL/2, TRACK_WIDTH),getAccelerationConstraint(MAX_ACCEL/4))
                .splineTo((new Vector2d(startPose.getX(),startPose.getY())).plus(new Vector2d(8, 26)), Math.toRadians(0),getVelocityConstraint(MAX_VEL/1.2, MAX_ANG_VEL/2, TRACK_WIDTH),getAccelerationConstraint(MAX_ACCEL/4))
                .addDisplacementMarker(()->{
                    robot.flipper.setPosition(v3Hardware.flipDown);
                })
                .forward(11)
                //drop pixels
                .splineToLinearHeading((startPose).plus(new Pose2d(-18, 13, Math.toRadians(-90))), Math.toRadians(180))
                .waitSeconds(0.5)
                .splineToLinearHeading((startPose).plus(new Pose2d(-20, 27.5, Math.toRadians(-90))), Math.toRadians(90))
                .build();
        TrajectorySequence right2 = drive.trajectorySequenceBuilder(right1.end())
                .back(10)
                .waitSeconds(0.1)
                .strafeRight(24.5)
                .turn(Math.toRadians(4))
                .waitSeconds(0.1)
                .back(86)
                //.splineToLinearHeading((startPose).plus(new Pose2d(-10, 52.5, Math.toRadians(-90))),Math.toRadians(0))
                //.splineToLinearHeading((startPose).plus(new Pose2d(68, 52.5, Math.toRadians(-90))),Math.toRadians(0))
                .lineToLinearHeading((startPose).plus(new Pose2d(78, 24, Math.toRadians(-90))))
                .build();
        TrajectorySequence left1 = drive.trajectorySequenceBuilder(startPose)
                .back(3)
                .splineToLinearHeading((startPose).plus(new Pose2d(-11, 19, Math.toRadians(0))),Math.toRadians(90))

                //drop pixels
                .forward(7)
                .turn(Math.toRadians(-90))
                .waitSeconds(0.2)
                .addDisplacementMarker(()->{
                    robot.flipper.setPosition(v3Hardware.flipDown);
                })
                .splineToLinearHeading((startPose).plus(new Pose2d(-20, 13, Math.toRadians(-90))), Math.toRadians(180))
                .waitSeconds(0.5)
                //.build();
                //TrajectorySequence left2 = drive.trajectorySequenceBuilder(left1.end())
                .splineToLinearHeading((startPose).plus(new Pose2d(-20, 27.5, Math.toRadians(-90))), Math.toRadians(90))
                .build();
        TrajectorySequence left2 = drive.trajectorySequenceBuilder(left1.end())
                .back(1)
                .strafeRight(24.5)
                //.splineToLinearHeading((startPose).plus(new Pose2d(-18, 50, Math.toRadians(-87))), Math.toRadians(90))
                .waitSeconds(0.5)
                .back(2)
                .splineToLinearHeading((startPose).plus(new Pose2d(0, 54.5, Math.toRadians(-87))),Math.toRadians(0))
                .back(1)
                .splineToLinearHeading((startPose).plus(new Pose2d(75, 54.5, Math.toRadians(-87))),Math.toRadians(0))
                .waitSeconds(0.1)
                .strafeLeft(12.5)
                .build();
        while (!isStarted()) {
            telemetry.addData("position", propPos("red", "far"));
            telemetry.addData("rightBlue", rightRedRatio);
            telemetry.addData("centerBlue", centerRedRatio);
            telemetry.addData("leftBlue", leftRedRatio);
            telemetry.addData("distance", robot.distanceTop.getDistance(DistanceUnit.MM));
            propPos =propPos("red", "far");
            telemetry.update();
        }
        waitForStart();
        drive.setPoseEstimate(startPose);
        camera.setPipeline(aprilTagDetectionPipeline2);
        v3autoBase.pipeline = "April Tag";
        if(propPos.equals("center")) {
            drive.followTrajectorySequence(center1);
        }
        else if(propPos.equals("left")){
            drive.followTrajectorySequence(left1);
        }
        else{
            drive.followTrajectorySequence(right1);
        }
        /*
        robot.flipper.setPosition(v3Hardware.flipUp +((v3Hardware.flipDown - v3Hardware.flipUp) / 1.3));
        sleep(500);
        robot.intake.setPower(v3Hardware.intakeSpeed);
        telemetry.addData("topDist",robot.distanceTop.getDistance(DistanceUnit.MM) );
        telemetry.addData("BottomDist",robot.distanceBottom.getDistance(DistanceUnit.MM) );
        telemetry.update();
        resetRuntime();
        while (getRuntime() < 5 && (robot.distanceTop.getDistance(DistanceUnit.MM) > 40 || robot.distanceBottom.getDistance(DistanceUnit.MM) > 40)) {
            robot.flipper.setPosition(v3Hardware.flipUp + (v3Hardware.flipDown - v3Hardware.flipUp) / (1.3 - (getRuntime() / 16)));
            telemetry.addData("topDist",robot.distanceTop.getDistance(DistanceUnit.MM));
            telemetry.addData("BottomDist",robot.distanceBottom.getDistance(DistanceUnit.MM));
            telemetry.update();
            if(getRuntime()>1&&getRuntime()<2){
                drive.setMotorPowers(0.2,0.2,-0.2,-0.2);
            }
            if(getRuntime()>2&&getRuntime()<3){
                drive.setMotorPowers(-0.2,-0.2,0.2,0.2);
            }
        }
        drive.setMotorPowers(0,0,0,0);*/
        robot.intake.setPower(v3Hardware.intakeSpeed);
        sleep(1000);
        robot.intake.setPower(-v3Hardware.intakeSpeed);
        sleep(500);
        robot.intake.setPower(v3Hardware.intakeSpeed);
        sleep(1000);
        robot.intake.setPower(-v3Hardware.intakeSpeed);
        sleep(1000);
        robot.intake.setPower(0);
        robot.flipper.setPosition(v3Hardware.flipUp);
        if(propPos.equals("center")) {
            drive.followTrajectorySequence(center2);
        }
        else if(propPos.equals("left")){
            drive.followTrajectorySequence(left2);
        }
        else{
            drive.followTrajectorySequence(right2);
        }
        telemetry.addData("Past 2nd path", "");
        telemetry.update();
        robot.intake.setPower(0);
        robot.shoulderPort.setPosition(v3Hardware.shoulderPortScore);
        robot.shoulderStar.setPosition(v3Hardware.shoulderStarScore);
        telemetry.addData("at shoulder", "");
        telemetry.update();
        resetRuntime();
        telemetry.addData("yaw", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        headingError = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)+84;
        telemetry.addData("yaw", headingError);
        telemetry.update();
        sleep(500);
        if(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)>5||imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)<-5) {
            drive.turn(Math.toRadians(-headingError));
        }
        else{
            drive.turn(Math.toRadians(10));
        }
        /*
        if(propPos.equals("center")) {
            updateAprilTagPosition(5);
        }
        else if(propPos.equals("left")){
            updateAprilTagPosition(4);
        }
        else{
            updateAprilTagPosition(6);
        }
        drive.setPoseEstimate(new Pose2d(0, 0, 0));
        TrajectorySequence aprilTagAdjustment = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-z+9,  x, 0))
                //.lineToLinearHeading(new Pose2d(-z+4, x-2, 0))
                .build();
        drive.followTrajectorySequence(aprilTagAdjustment);
        /*
        if(propPos.equals("left")){
            drive.strafeRight(2);}
        else if(propPos.equals("right")){
            drive.strafeLeft(2);}

         */
        camera.closeCameraDevice();
        /*robot.wrist.setPosition(v3Hardware.wristPort);
        sleep(500);
        robot.wrist.setPosition(v3Hardware.extendyBoiExtend);
        sleep(1000);
        robot.wrist.setPosition(v3Hardware.wristDown);
        sleep(500);*/
        resetRuntime();
        while(!robot.handTS.isPressed()&& getRuntime()<3){
            drive.setMotorPowers(-0.2,-0.2,-0.2,-0.2);}
        drive.setMotorPowers(0,0,0,0);
        robot.portArm.setPower(0.5);
        robot.starArm.setPower(0.5);
        sleep(300);
        robot.door.setPosition(v3Hardware.doorOpen);
        robot.portArm.setPower(0);
        robot.starArm.setPower(0);
        sleep(300);
        robot.starArm.setPower(1);
        robot.portArm.setPower(1);
        sleep(700);
        robot.starArm.setPower(0);
        robot.portArm.setPower(0);
        /*robot.wrist.setPosition(v3Hardware.wristPort);
        sleep(500);
        robot.wrist.setPosition(v3Hardware.extendyBoiRetract);
        sleep(1000);
        robot.wrist.setPosition(v3Hardware.wristDown);*/
        robot.shoulderPort.setPosition(v3Hardware.shoulderPortDown);
        robot.shoulderStar.setPosition(v3Hardware.shoulderStarDown);
        sleep(500);
        robot.portArm.setPower(-1);
        robot.starArm.setPower(-1);
        sleep(2000);
    }
}