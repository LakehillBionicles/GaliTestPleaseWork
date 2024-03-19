package org.firstinspires.ftc.teamcode.GaliV3.v3Auto;

import static org.firstinspires.ftc.teamcode.Vision.BlueColorProcessor.blueTolerance;
import static org.firstinspires.ftc.teamcode.Vision.BlueColorProcessor.centerBlueRatio;
import static org.firstinspires.ftc.teamcode.Vision.BlueColorProcessor.leftBlueRatio;
import static org.firstinspires.ftc.teamcode.Vision.BlueColorProcessor.rightBlueRatio;
import static org.firstinspires.ftc.teamcode.Vision.RedColorProcessor.centerRedRatio;
import static org.firstinspires.ftc.teamcode.Vision.RedColorProcessor.leftRedRatio;
import static org.firstinspires.ftc.teamcode.Vision.RedColorProcessor.redTolerance;
import static org.firstinspires.ftc.teamcode.Vision.RedColorProcessor.rightRedRatio;

import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.GaliV3.v3Hardware;
import org.firstinspires.ftc.teamcode.Vision.AprilTagDetectionPipeline2;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

public class visionFunctions extends v3autoBase {
    public static String pipeline;
    public v3Hardware robot = new v3Hardware();
    public org.firstinspires.ftc.teamcode.Vision.RedColorProcessor RedColorProcessor;
    static AprilTagDetectionPipeline2 aprilTagDetectionPipeline2;
    public org.firstinspires.ftc.teamcode.Vision.BlueColorProcessor BlueColorProcessor;
    public String propPos = "notSeen";
    public static String robotPosition = "notSeen";
    public BNO055IMU imu;
    private String webcam1 = "Webcam 1";
    static double x = 0;
    static double y = 0;
    static double yaw = 0;
    static double z = 0;
    public Orientation robotTheta;
    static final double FEET_PER_METER = 3.28084;
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;
    double tagsize = 0.166;
    static int numFramesWithoutDetection = 0;

    static final float DECIMATION_HIGH = 3;
    static final float DECIMATION_LOW = 2;
    static final float THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f;
    static final int THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION = 4;
    OpenCvCamera camera;
    static boolean stayInLoop = true;
    static double targetZ = 0;
    static double targetX = 0;
    static double targetYaw = 0;


    public String propPos(String color, String placement) {
        if (placement.equals("close")) {
            robotPosition = "close";
        }
        if (placement.equals("far")) {
            robotPosition = "far";
        }
        if (color.equals("blue")) {
            if (placement.equals("close")) {
                if (leftBlueRatio > blueTolerance && leftBlueRatio > centerBlueRatio) {
                    propPos = "left";
                } else if (centerBlueRatio > blueTolerance && centerBlueRatio > leftBlueRatio) {
                    propPos = "center";
                } else {
                    propPos = "right";
                }
            } else {
                if (rightBlueRatio > blueTolerance && rightBlueRatio > centerBlueRatio) {
                    propPos = "right";
                } else if (centerBlueRatio > blueTolerance && centerBlueRatio > rightBlueRatio) {
                    propPos = "center";
                } else {
                    propPos = "left";
                }
            }
        } else {
            if (placement.equals("close")) {
                if (rightRedRatio > redTolerance && rightRedRatio > centerRedRatio) {
                    propPos = "right";
                } else if (centerRedRatio > redTolerance && centerRedRatio > rightRedRatio) {
                    propPos = "center";
                } else {
                    propPos = "left";
                }
            } else {
                if (leftRedRatio > redTolerance && leftRedRatio > centerRedRatio) {
                    propPos = "left";
                } else if (centerRedRatio > redTolerance && centerRedRatio > leftRedRatio) {
                    propPos = "center";
                } else {
                    propPos = "right";
                }
            }
        }
        return propPos;
    }

    public void cameraStartup(String color) {
    if(color.equals("red")||color.equals("Red")) {
        pipeline = "propRed";
    }
    else{
        pipeline = "propBlue";
    }
    int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
    camera =OpenCvCameraFactory.getInstance().

    createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"),cameraMonitorViewId);
    aprilTagDetectionPipeline2 =new AprilTagDetectionPipeline2(tagsize, fx, fy, cx, cy);

            camera.setPipeline(aprilTagDetectionPipeline2);
            camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()

    {
        @Override
        public void onOpened ()
        {
            camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
        }

        @Override
        public void onError ( int errorCode)
        {

        }
    });
}
public static void updateAprilTagPos(){
        stayInLoop = true;
    while(stayInLoop) {
        ArrayList<AprilTagDetection> detections = aprilTagDetectionPipeline2.getDetectionsUpdate();
        // If there's been a new frame...
        if (detections != null) {
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
                    x = detection.pose.x * FEET_PER_METER;
                    y = detection.pose.y * FEET_PER_METER;
                    z = detection.pose.z * FEET_PER_METER;
                    yaw = detection.pose.R.get(2,1);
                    //telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
                    //telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
                    //telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
                    stayInLoop = false;
                }
            }
        }
        //sleep(20);
    }
}
public void switchToAprilTagPipeline(){
        pipeline = "aprilTags";
}
public static double getX(){
        return x;
}
public static double getY(){
        return y;
    }
    public static double getZ(){
        return z;
    }
    public static double getYaw(){
        return yaw;
    }
    public static double getTargetZ(){
        targetZ = (-getZ()+(getZ()*Math.cos(getYaw())*12));
        return targetZ;
    }
    public static double getTargetX() {
        targetX = ((4*Math.sin(getYaw()))+getX())*12;
        return targetX;
    }
    public static double getTargetYaw(){
        targetYaw = -getYaw();
        return targetYaw;
    }
}
