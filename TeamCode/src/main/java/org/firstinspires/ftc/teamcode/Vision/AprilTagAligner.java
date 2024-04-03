package org.firstinspires.ftc.teamcode.Vision;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
@TeleOp
@Disabled

public class AprilTagAligner extends LinearOpMode{
        OpenCvCamera camera;
        AprilTagDetectionPipeline2 aprilTagDetectionPipeline;

        static final double FEET_PER_METER = 3.28084;

        // Lens intrinsics
        // UNITS ARE PIXELS
        // NOTE: this calibration is for the C920 webcam at 800x448.
        // You will need to do your own calibration for other configurations!

        //c920 intrinsics
    /*
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

     */
        //c270 intrinsics
    /*
    double fx = 1078.03779;
    double fy = 1084.50988;
    double cx = 580.850545;
    double cy = 245.959325;

     */
        //It might also be this for c270
        double fx = 1430;
        double fy = 1430;
        double cx = 480;
        double cy = 620;

        // UNITS ARE METERS
        double tagsize = 0.166;

        int numFramesWithoutDetection = 0;

        final float DECIMATION_HIGH = 3;
        final float DECIMATION_LOW = 2;
        final float THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f;
        final int THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION = 4;
        public double aprilTagXPosition = 198;//Used to check if we have seen april tag

        @SuppressLint("DefaultLocale")
        @Override
        public void runOpMode()
        {
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
            camera.setPipeline(aprilTagDetectionPipeline);
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
            while (!opModeIsActive()){
                    // Calling getDetectionsUpdate() will only return an object if there was a new frame
                    // processed since the last time we called it. Otherwise, it will return null. This
                    // enables us to only run logic when there has been a new frame, as opposed to the
                    // getLatestDetections() method which will always return an object.
                    ArrayList<AprilTagDetection> detections = aprilTagDetectionPipeline.getDetectionsUpdate();
                    // If there's been a new frame...
                    if(detections != null) {
                        //telemetry.addData("FPS", camera.getFps());
                        //telemetry.addData("Overhead ms", camera.getOverheadTimeMs());
                        //telemetry.addData("Pipeline ms", camera.getPipelineTimeMs());
                        // If we don't see any tags
                        if(detections.size() == 0)
                        {numFramesWithoutDetection++;
                            // If we haven't seen a tag for a few frames, lower the decimation
                            // so we can hopefully pick one up if we're e.g. far back
                            if(numFramesWithoutDetection >= THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION)
                            {aprilTagDetectionPipeline.setDecimation(DECIMATION_LOW);}}
                        // We do see tags!
                        else
                        {numFramesWithoutDetection = 0;
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
                                aprilTagXPosition = detection.pose.x*FEET_PER_METER*12;
                                telemetry.addData("Translation X", aprilTagXPosition);
                                telemetry.update();}}}sleep(20);}}}
