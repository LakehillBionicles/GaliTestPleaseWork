package org.firstinspires.ftc.teamcode.Vision;
import org.firstinspires.ftc.teamcode.GaliV3.v3Auto.visionFunctions;
import static org.firstinspires.ftc.teamcode.Vision.BlueColorProcessor.centerBlueRatio;
import static org.firstinspires.ftc.teamcode.Vision.BlueColorProcessor.leftBlueRatio;
import static org.firstinspires.ftc.teamcode.Vision.BlueColorProcessor.rightBlueRatio;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.GaliV3.v3Auto.v3autoBase;
import org.firstinspires.ftc.teamcode.GaliV3.v3Hardware;
import org.firstinspires.ftc.teamcode.GaliV3.v3Roadrunner.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import static org.firstinspires.ftc.teamcode.GaliV3.v3Roadrunner.drive.SampleMecanumDrive.getVelocityConstraint;
import static org.firstinspires.ftc.teamcode.GaliV3.v3Roadrunner.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.GaliV3.v3Roadrunner.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.GaliV3.v3Roadrunner.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.GaliV3.v3Roadrunner.drive.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.GaliV3.v3Roadrunner.drive.SampleMecanumDrive.getAccelerationConstraint;

import org.firstinspires.ftc.teamcode.GaliV3.v3Roadrunner.drive.SampleMecanumDrive;

@Autonomous
@Disabled
public class FancyAprilTagIntegrationTest extends v3autoBase {
        public v3Hardware robot = new v3Hardware();
        public org.firstinspires.ftc.teamcode.Vision.RedColorProcessor RedColorProcessor;
        public org.firstinspires.ftc.teamcode.Vision.BlueColorProcessor BlueColorProcessor;
        public String  propPos = "notSeen";
        public static String pipeline = "";
        public static String robotPosition = "notSeen";
        public double x = 0;
        public double y = 0;
        public double z = 0;
        public double yaw = 0;

        public BNO055IMU imu;
        private String webcam = "Webcam ";
        public Orientation robotTheta;
        static final double FEET_PER_METER = 3.28084;
        double fx = 578.272;
        double fy = 578.272;
        double cx = 402.145;
        double cy = 221.506;
        double tagsize = 0.05;
        Pose2d startPose = new Pose2d(-38, 61, Math.toRadians(-90));
        public static double forWard = 0;
        public static double turn1 = 0;
        AprilTagDetectionPipeline aprilTagDetectionPipeline2= new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
        int numFramesWithoutDetection = 0;

        final float DECIMATION_HIGH = 3;
        final float DECIMATION_LOW = 2;
        final float THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f;
        final int THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION = 4;
        OpenCvCamera camera;
        OpenCvCamera camera1;
        boolean stayInLoop = true;
        double i = 0;
        @Override
        public void runOpMode() {
            super.runOpMode();
            SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
            pipeline = "propRed";
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
            aprilTagDetectionPipeline2 = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

            camera.setPipeline(aprilTagDetectionPipeline2);
            camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
            {
                @Override
                public void onOpened()
                {
                    camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
                }

                @Override
                public void onError(int errorCode)
                {

                }
            });
            while (!isStarted()) {
                telemetry.addData("position", propPos("red", "far"));
                telemetry.addData("rightBlue", rightBlueRatio);
                telemetry.addData("centerBlue", centerBlueRatio);
                telemetry.addData("leftBlue", leftBlueRatio);
                telemetry.update();
            }
            pipeline="AprilTags";
            telemetry.addData("setPipeline", "sí");
            telemetry.update();
            if (opModeIsActive()){
                telemetry.addData("yay!", "");
                telemetry.update();
                visionFunctions.updateAprilTagPos();
                startPose = drive.getPoseEstimate();
                TrajectorySequence aprilTagAdjustment = drive.trajectorySequenceBuilder(startPose)
                        .lineToLinearHeading(new Pose2d((visionFunctions.getTargetZ()), visionFunctions.getTargetX(),(Math.toRadians(visionFunctions.getTargetYaw()))), getVelocityConstraint(MAX_VEL/3,MAX_ANG_VEL/3,TRACK_WIDTH), getAccelerationConstraint(MAX_ACCEL/2))
                        //.lineToLinearHeading(new Pose2d(((-z+z*Math.cos(yaw))*12), ((4*Math.sin(yaw)+x)*12),(Math.toRadians(-yaw))), getVelocityConstraint(MAX_VEL/3,MAX_ANG_VEL/3,TRACK_WIDTH), getAccelerationConstraint(MAX_ACCEL/2))
                        .build();
                drive.followTrajectorySequence(aprilTagAdjustment);
                camera.closeCameraDevice();
                sleep(5000);}}
        public static String getPipeline(){
            return pipeline;
        }
    }
