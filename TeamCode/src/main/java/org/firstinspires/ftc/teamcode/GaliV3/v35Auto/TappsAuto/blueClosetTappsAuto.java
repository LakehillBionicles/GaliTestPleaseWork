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
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
@Autonomous
public class blueClosetTappsAuto extends v3autoBase {
    Pose2d startPose = new Pose2d(-38, 61, Math.toRadians(90));
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
        v3autoBase.pipeline = "propBlue";
        v3autoBase.robotPosition = "close";
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
                .back(13)
                .splineTo((new Vector2d(startPose.getX(),startPose.getY())).plus(new Vector2d(2, -28)), Math.toRadians(-90))
                //drop pixels
                .addDisplacementMarker(()->{
                    robot.shoulderPort.setPosition(v3Hardware.shoulderPortScore);
                    robot.shoulderStar.setPosition(v3Hardware.shoulderStarScore);
                })
                .forward(10)
                .turn(Math.toRadians(90))
                .waitSeconds(0.2)
                .back(1)
                .splineToLinearHeading((startPose).plus(new Pose2d(33, -29, Math.toRadians(90))), Math.toRadians(0))
                .build();
        TrajectorySequence left1 = drive.trajectorySequenceBuilder(startPose)
                .back(5)
                .splineToLinearHeading((startPose).plus(new Pose2d(9.5, -17, Math.toRadians(0))),Math.toRadians(-90))
                .addDisplacementMarker(()->{
                    robot.shoulderPort.setPosition(v3Hardware.shoulderPortScore);
                    robot.shoulderStar.setPosition(v3Hardware.shoulderStarScore);
                })
                //drop pixels
                .forward(8)
                .turn(Math.toRadians(90))
                .back(4)
                .splineToLinearHeading((startPose).plus(new Pose2d(34, -20, Math.toRadians(90))),Math.toRadians(0))
                .build();
        TrajectorySequence right1 = drive.trajectorySequenceBuilder(startPose)
                .back(1)
                //.waitSeconds(0.1)
                //.strafeLeft(10)
                //.waitSeconds(0.1)
                //.back(2)
                .splineTo((new Vector2d(startPose.getX(),startPose.getY())).plus(new Vector2d(2, -18)), Math.toRadians(-90), getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH),getAccelerationConstraint(MAX_ACCEL/1))
                .splineTo((new Vector2d(startPose.getX(),startPose.getY())).plus(new Vector2d(2, -26)), Math.toRadians(180), getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH),getAccelerationConstraint(MAX_ACCEL/1))
                .splineTo((new Vector2d(startPose.getX(),startPose.getY())).plus(new Vector2d(-9, -29)), Math.toRadians(180), getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH),getAccelerationConstraint(MAX_ACCEL/1))
                .addDisplacementMarker(()->{
                    robot.shoulderPort.setPosition(v3Hardware.shoulderPortScore);
                    robot.shoulderStar.setPosition(v3Hardware.shoulderStarScore);
                })
                .forward(11)
                .turn(Math.toRadians(180))
                .back(2)
                .splineToLinearHeading((startPose).plus(new Pose2d(33, -34, Math.toRadians(90))),Math.toRadians(0))
                .build();

        while (!isStarted()) {
            telemetry.addData("position", propPos("blue", "close"));
            telemetry.addData("rightBlue", rightRedRatio);
            telemetry.addData("centerBlue", centerRedRatio);
            telemetry.addData("leftBlue", leftRedRatio);
            telemetry.addData("distance", robot.distanceTop.getDistance(DistanceUnit.MM));
            telemetry.addData("blinker left", robot.blinkerPort.getDistance(DistanceUnit.MM));
            telemetry.addData("blinker right", robot.blinkerStar.getDistance(DistanceUnit.MM));
            propPos =propPos("blue", "close");
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
        if(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)>5||imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)<-5) {
            drive.turn(Math.toRadians(-headingError));
        }
        else if(!propPos.equals("left")){
            drive.turn(Math.toRadians(10));}

         */
        sleep(100);
        drive.turn(Math.toRadians(drive.getPoseEstimate().getHeading()));
        sleep(100);
        resetRuntime();
        while(!robot.handTS.isPressed()&& getRuntime()<3){
            drive.setMotorPowers(-0.2,-0.2,-0.2,-0.2);}
        drive.setMotorPowers(0,0,0,0);
        if(propPos.equals("left")){
            int i=0;
            double gatekeeper = Math.min(robot.blinkerStar.getDistance(DistanceUnit.MM), robot.blinkerPort.getDistance(DistanceUnit.MM))+50;
            while(robot.blinkerStar.getDistance(DistanceUnit.MM) <300 && robot.blinkerPort.getDistance(DistanceUnit.MM) <300 && i<12 ){
                /*
                drive.strafeRight(4);
                drive.turn(Math.toRadians(drive.getPoseEstimate().getHeading()));
                i++;

                 */
                drive.setMotorPowers(0.5,-0.5,0.5,-0.5);
                telemetry.addData("heading",drive.getPoseEstimate().getHeading());
                telemetry.addData("blinker left", robot.blinkerPort.getDistance(DistanceUnit.MM));
                telemetry.addData("blinker right", robot.blinkerStar.getDistance(DistanceUnit.MM));
                telemetry.update();
            }
            i=0;
            while(robot.blinkerStar.getDistance(DistanceUnit.MM) >300 && robot.blinkerPort.getDistance(DistanceUnit.MM) <300 && i<12 ){
                /*
                drive.strafeLeft(1);
                drive.turn(-Math.toRadians(-drive.getPoseEstimate().getHeading()));
                i++;
                 */
                drive.setMotorPowers(-0.5,0.5,-0.5,0.5);
                telemetry.addData("heading",drive.getPoseEstimate().getHeading());
                telemetry.addData("blinker left", robot.blinkerPort.getDistance(DistanceUnit.MM));
                telemetry.addData("blinker right", robot.blinkerStar.getDistance(DistanceUnit.MM));
                telemetry.update();
            }
            //drive.turn(-Math.toRadians(-drive.getPoseEstimate().getHeading()+90));
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
        telemetry.addData("Past 2nd path", "");
        telemetry.update();
        robot.shoulderPort.setPosition(v3Hardware.shoulderPortScore);
        robot.shoulderStar.setPosition(v3Hardware.shoulderStarScore);
        telemetry.addData("at shoulder", "");
        telemetry.update();
        resetRuntime();
        telemetry.addData("yaw", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        headingError = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)-90;
        telemetry.addData("yaw", headingError);
        telemetry.update();
        sleep(500);
        camera.closeCameraDevice();
        resetRuntime();
        while(!robot.handTS.isPressed()&& getRuntime()<3){
            drive.setMotorPowers(-0.2,-0.2,-0.2,-0.2);}
        drive.setMotorPowers(0,0,0,0);
        robot.door.setPosition(v3Hardware.doorOpen);
        sleep(1000);
        drive.forward(10);
        sleep(500);
        drive.strafeRight(30);
        sleep(500);
        robot.shoulderPort.setPosition(v3Hardware.shoulderPortDown);
        robot.shoulderStar.setPosition(v3Hardware.shoulderStarDown);
        drive.back(10);
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