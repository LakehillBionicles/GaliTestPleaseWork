package org.firstinspires.ftc.teamcode.Vision;

import static org.firstinspires.ftc.teamcode.Vision.BlueColorProcessor.centerBlueRatio;
import static org.firstinspires.ftc.teamcode.Vision.BlueColorProcessor.leftBlueRatio;
import static org.firstinspires.ftc.teamcode.Vision.BlueColorProcessor.rightBlueRatio;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.GaliV3.v3Auto.v3autoBase;
import org.firstinspires.ftc.teamcode.GaliV3.v3Hardware;
import org.firstinspires.ftc.teamcode.GaliV3.v3Roadrunner.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import static org.firstinspires.ftc.teamcode.GaliV3.v3Roadrunner.drive.SampleMecanumDrive.getVelocityConstraint;
import static org.firstinspires.ftc.teamcode.Vision.BlueColorProcessor.centerBlueRatio;
import static org.firstinspires.ftc.teamcode.Vision.BlueColorProcessor.leftBlueRatio;
import static org.firstinspires.ftc.teamcode.Vision.BlueColorProcessor.rightBlueRatio;
import static org.firstinspires.ftc.teamcode.GaliV3.v3Roadrunner.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.GaliV3.v3Roadrunner.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.GaliV3.v3Roadrunner.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.GaliV3.v3Roadrunner.drive.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.GaliV3.v3Roadrunner.drive.SampleMecanumDrive.getAccelerationConstraint;
import static org.firstinspires.ftc.teamcode.GaliV3.v3Roadrunner.drive.SampleMecanumDrive.getVelocityConstraint;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.GaliV3.v3Auto.v3autoBase;
import org.firstinspires.ftc.teamcode.GaliV3.v3Hardware;
import org.firstinspires.ftc.teamcode.GaliV3.v3Roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.GaliV3.v3Roadrunner.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;

import java.time.YearMonth;
import java.util.ArrayList;
@Autonomous
public class AprilTagIntegrationTest extends v3autoBase {
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
        */
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
        while (!isStarted()
        ) {
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
                // Calling getDetectionsUpdate() will only return an object if there was a new frame
                // processed since the last time we called it. Otherwise, it will return null. This
                // enables us to only run logic when there has been a new frame, as opposed to the
                // getLatestDetections() method which will always return an object.
                while(stayInLoop) {
                    ArrayList<AprilTagDetection> detections = aprilTagDetectionPipeline2.getDetectionsUpdate();
                    // If there's been a new frame...
                    if (detections != null) {
                        telemetry.addData("wohooo", "");
                        telemetry.update();
                    /*telemetry.addData("FPS", camera.getFps());
                    telemetry.addData("Overhead ms", camera.getOverheadTimeMs());
                    telemetry.addData("Pipeline ms", camera.getPipelineTimeMs());

                     */

                        // If we don't see any tags
                        if (detections.size() == 0) {
                            numFramesWithoutDetection++;

                            // If we haven't seen a tag for a few frames, lower the decimation
                            // so we can hopefully pick one up if we're e.g. far back
                            if (numFramesWithoutDetection >= THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION) {
                                aprilTagDetectionPipeline2.setDecimation(DECIMATION_LOW);
                            }
                        }
                        // We do see tags!
                        else {

                            numFramesWithoutDetection = 0;

                            // If the target is within 1 meter, turn on high decimation to
                            // increase the frame rate
                            if (detections.get(0).pose.z < THRESHOLD_HIGH_DECIMATION_RANGE_METERS) {
                                aprilTagDetectionPipeline2.setDecimation(DECIMATION_HIGH);
                            }

                            for (AprilTagDetection detection : detections) {
                                telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x * FEET_PER_METER));
                                x = detection.pose.x * FEET_PER_METER;
                                telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y * FEET_PER_METER));
                                y = detection.pose.y * FEET_PER_METER;
                                telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z * FEET_PER_METER));
                                z = detection.pose.z * FEET_PER_METER;
                                yaw = detection.pose.R.get(2,1);
                                telemetry.addLine(String.format("yawRadians", detection.pose.R.get(2,1)));
                                telemetry.addLine(String.format("yawDegrees", Math.toRadians(detection.pose.R.get(2,1))));
                                //telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
                                //telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
                                //telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
                                stayInLoop = false;
                            }
                        }

                        telemetry.update();
                    }
                    sleep(20);
                }
                //TrajectorySequence aprilTagAdjustment = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        //.lineToLinearHeading(new Pose2d(-z, x, robot.imu.getRobotOrientation));
                        //.build();
                //drive.followTrajectorySequence(aprilTagAdjustment);
                telemetry.addData("heading",drive.getRawExternalHeading());
                telemetry.update();
                camera.closeCameraDevice();
                sleep(5000);}}
public static String getPipeline(){
 return pipeline;
}
}
