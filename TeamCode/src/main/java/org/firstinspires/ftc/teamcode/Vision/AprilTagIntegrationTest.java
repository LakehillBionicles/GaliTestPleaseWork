package org.firstinspires.ftc.teamcode.Vision;

import static org.firstinspires.ftc.teamcode.Vision.BlueColorProcessor.centerBlueRatio;
import static org.firstinspires.ftc.teamcode.Vision.BlueColorProcessor.leftBlueRatio;
import static org.firstinspires.ftc.teamcode.Vision.BlueColorProcessor.rightBlueRatio;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.GaliV3.v3Auto.v3autoBase;
import org.firstinspires.ftc.teamcode.GaliV3.v3Hardware;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
@Autonomous
public class AprilTagIntegrationTest extends v3autoBase {
        public v3Hardware robot = new v3Hardware();
        public org.firstinspires.ftc.teamcode.Vision.RedColorProcessor RedColorProcessor;
        AprilTagDetectionPipeline aprilTagDetectionPipeline;
        public org.firstinspires.ftc.teamcode.Vision.BlueColorProcessor BlueColorProcessor;
        public String  propPos = "notSeen";
        public static String robotPosition = "notSeen";

        public BNO055IMU imu;
        private String webcam1 = "Webcam 1";
        public Orientation robotTheta;
        static final double FEET_PER_METER = 3.28084;
        double fx = 578.272;
        double fy = 578.272;
        double cx = 402.145;
        double cy = 221.506;
        double tagsize = 0.166;
        int numFramesWithoutDetection = 0;

        final float DECIMATION_HIGH = 3;
        final float DECIMATION_LOW = 2;
        final float THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f;
        final int THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION = 4;
        OpenCvCamera camera;
        OpenCvCamera camera1;
        double i = 0;
    @Override
    public void runOpMode() {
        super.runOpMode();
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
        if(gamepad1.b){
            int cameraMonitorViewId1 = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            telemetry.addData("cameraMonitorViewId", "sí");
            telemetry.update();
            camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId1);
            camera1 = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId1);
            telemetry.addData("OpenCvCameraFactory", "sí");
            telemetry.update();
            aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
            telemetry.addData("aprilTagDetectionPipeline", "sí");
            telemetry.update();
            camera.closeCameraDevice();
            telemetry.addData("closeCameraDevice", "sí");
            telemetry.update();
            camera1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
                    camera1.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                }
                @Override
                public void onError(int errorCode) {
                }
            });
            camera1.setPipeline(aprilTagDetectionPipeline);
            telemetry.addData("openCameraDeviceAsync", "sí");
            telemetry.update();
            telemetry.addData("setPipeline", "sí");
            telemetry.update();
            while (!opModeIsActive()&& !isStopRequested()){
                i++;
                // Calling getDetectionsUpdate() will only return an object if there was a new frame
                // processed since the last time we called it. Otherwise, it will return null. This
                // enables us to only run logic when there has been a new frame, as opposed to the
                // getLatestDetections() method which will always return an object.
                ArrayList<AprilTagDetection> detections = aprilTagDetectionPipeline.getDetectionsUpdate();
                telemetry.addData("While", i);
                telemetry.addData(String.valueOf(detections), "gracias");
                telemetry.update();
                // If there's been a new frame...
                if(detections != null) {
                    telemetry.addData("While1", i);
                    telemetry.update();
                    //telemetry.addData("FPS", camera.getFps());
                    //telemetry.addData("Overhead ms", camera.getOverheadTimeMs());
                    //telemetry.addData("Pipeline ms", camera.getPipelineTimeMs());
                    // If we don't see any tags
                    if(detections.size() == 0)
                    {numFramesWithoutDetection++;
                        telemetry.addData("While2", "sí");
                        telemetry.update();
                        // If we haven't seen a tag for a few frames, lower the decimation
                        // so we can hopefully pick one up if we're e.g. far back
                        if(numFramesWithoutDetection >= THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION)
                        {aprilTagDetectionPipeline.setDecimation(DECIMATION_LOW);}}
                    // We do see tags!
                    else
                    {numFramesWithoutDetection = 0;
                        telemetry.addData("While3", "sí");
                        telemetry.update();
                        // If the target is within 1 meter, turn on high decimation to
                        // increase the frame rate
                        if(detections.get(0).pose.z < THRESHOLD_HIGH_DECIMATION_RANGE_METERS)
                        {aprilTagDetectionPipeline.setDecimation(DECIMATION_HIGH);
                        }for(AprilTagDetection detection : detections) {
                        telemetry.addData("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER*12);
                        telemetry.addData("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER*12);
                        telemetry.addData("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER*12);
                        telemetry.addData("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.R.get(1,3)));

                        //telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.R.get(2,1))));
                        //telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.R.get(3,1))));
                        telemetry.update();}}}sleep(20);}
            stop();
        }}}
