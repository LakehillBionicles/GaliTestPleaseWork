package org.firstinspires.ftc.teamcode.GaliV3.v3Auto;

import static org.firstinspires.ftc.teamcode.GaliV3.v3Auto.visionFunctions.aprilTagDetectionPipeline2;
import static org.firstinspires.ftc.teamcode.GaliV3.v3Hardware.doorClosed;
import static org.firstinspires.ftc.teamcode.GaliV3.v3Hardware.elbowNorminal;
import static org.firstinspires.ftc.teamcode.GaliV3.v3Hardware.elbowPort;
import static org.firstinspires.ftc.teamcode.GaliV3.v3Hardware.extendyBoiRetract;
import static org.firstinspires.ftc.teamcode.GaliV3.v3Hardware.shoulderPortDown;
import static org.firstinspires.ftc.teamcode.GaliV3.v3Hardware.shoulderPortScore;
import static org.firstinspires.ftc.teamcode.GaliV3.v3Hardware.shoulderStarDown;
import static org.firstinspires.ftc.teamcode.GaliV3.v3Hardware.shoulderStarScore;
import static org.firstinspires.ftc.teamcode.GaliV3.v3Hardware.wristDown;
import static org.firstinspires.ftc.teamcode.GaliV3.v3Hardware.wristPort;
import static org.firstinspires.ftc.teamcode.Vision.BlueColorProcessor.blueTolerance;
import static org.firstinspires.ftc.teamcode.Vision.BlueColorProcessor.centerBlueRatio;
import static org.firstinspires.ftc.teamcode.Vision.BlueColorProcessor.leftBlueRatio;
import static org.firstinspires.ftc.teamcode.Vision.BlueColorProcessor.rightBlueRatio;
import static org.firstinspires.ftc.teamcode.Vision.RedColorProcessor.centerRedRatio;
import static org.firstinspires.ftc.teamcode.Vision.RedColorProcessor.leftRedRatio;
import static org.firstinspires.ftc.teamcode.Vision.RedColorProcessor.redTolerance;
import static org.firstinspires.ftc.teamcode.Vision.RedColorProcessor.rightRedRatio;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.GaliV3.v3Hardware;
import org.firstinspires.ftc.teamcode.Vision.AprilTagDetectionPipeline2;
import org.firstinspires.ftc.teamcode.Vision.BlueColorProcessor;
import org.firstinspires.ftc.teamcode.Vision.RedColorProcessor;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Objects;

public class v3autoBase extends LinearOpMode {
    public static String pipeline = "April Tags";
    public v3Hardware robot = new v3Hardware();
    public org.firstinspires.ftc.teamcode.Vision.RedColorProcessor RedColorProcessor;
    AprilTagDetectionPipeline2 aprilTagDetectionPipeline;
    public org.firstinspires.ftc.teamcode.Vision.BlueColorProcessor BlueColorProcessor;
    public String  propPos = "notSeen";
    public static String robotPosition = "notSeen";
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
    public double x = 0;
    public double y = 0;
    public double z = 0;
    public double yaw = 0;
    public void runOpMode(){
        robot.init(hardwareMap);
        aprilTagDetectionPipeline2 = new AprilTagDetectionPipeline2(tagsize, fx, fy, cx, cy);
        robot.door.setPosition(v3Hardware.doorClosed);
        robot.extendyBoi.setPosition(v3Hardware.extendyBoiRetract);
        robot.elbow.setPosition(v3Hardware.elbowNorminal);
        robot.shoulderStar.setPosition(v3Hardware.shoulderStarDown);
        robot.shoulderPort.setPosition(v3Hardware.shoulderPortDown);
        robot.wrist.setPosition(v3Hardware.wristDown);
    }
    public void cameraStartup(String cameraName){
        aprilTagDetectionPipeline2 = new AprilTagDetectionPipeline2(tagsize, fx, fy, cx, cy);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, cameraName), cameraMonitorViewId);
    }
    public void propDetection(String color){
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode) {
            }
        });
        if(Objects.equals(color, "blue")) {
            BlueColorProcessor = new BlueColorProcessor();
            pipeline="propBlue";
            //camera.setPipeline(BlueColorProcessor);
        }
        if(Objects.equals(color, "red")){
            RedColorProcessor = new RedColorProcessor();
            pipeline="propRed";
            //camera.setPipeline(RedColorProcessor);
        }
        camera.setPipeline(aprilTagDetectionPipeline);
    }
    public String propPos(String color, String placement){
        if(placement.equals("close")){
            robotPosition = "close";
        }
        if(placement.equals("far")){
            robotPosition = "far";
        }
        if(color.equals("blue")){
            if(placement.equals("close")){
                if(leftBlueRatio>blueTolerance && leftBlueRatio> centerBlueRatio){
                    propPos = "left";
                }
                else if(centerBlueRatio>blueTolerance && centerBlueRatio> leftBlueRatio){
                    propPos = "center";
                }
                else{
                    propPos = "right";
                }
            }
            else{
                if(rightBlueRatio>blueTolerance && rightBlueRatio> centerBlueRatio){
                    propPos = "right";
                }
                else if(centerBlueRatio>blueTolerance && centerBlueRatio> rightBlueRatio){
                    propPos = "center";
                }
                else{
                    propPos = "left";
                }
            }
        }
        else{
            if(placement.equals("close")){
                if(rightRedRatio>redTolerance && rightRedRatio> centerRedRatio){
                    propPos = "right";
                }
                else if(centerRedRatio>redTolerance && centerRedRatio> rightRedRatio){
                    propPos = "center";
                }
                else{
                    propPos = "left";
                }
            }
            else{
                if(leftRedRatio>redTolerance&& leftRedRatio>centerRedRatio){
                    propPos = "left";
                }
                else if(centerRedRatio>redTolerance&&centerRedRatio>leftRedRatio){
                    propPos = "center";
                }
                else{
                    propPos = "right";
                }
            }
        }
       return propPos;
    }
    public void updateAprilTagPosition(double aprilTagId){
        boolean stayInLoop = true;
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
                        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
                        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
                        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
                        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
                        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.R.get(2,1))));//bueno
                        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.R.get(1,1))));
                        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.R.get(0,2))));
                        if(detection.id == aprilTagId) {
                            x = detection.pose.x * FEET_PER_METER * 12;
                            y = detection.pose.y * FEET_PER_METER * 12;
                            z = detection.pose.z * FEET_PER_METER * 12;
                            yaw = detection.pose.R.get(2, 1);
                            //telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
                            //telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
                            //telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
                            stayInLoop = false;
                        }
                    }
                    telemetry.update();
                }
            }
            sleep(20);
        }
    }
    public boolean checkBoardForRobot(double aprilTagId){
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
                        if(detection.id == aprilTagId) {
                            x = detection.pose.x * FEET_PER_METER * 12;
                            y = detection.pose.y * FEET_PER_METER * 12;
                            z = detection.pose.z * FEET_PER_METER * 12;
                            yaw = detection.pose.R.get(2, 1);
                            return false;
                            //telemetry.addLine(String.format("Rotatio
                            //n Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
                            //telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
                            //telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
                        }
                    }
                    if (detections.size() >= 3) {
                        return false;
                    }
                    telemetry.addData("detections size",detections.size());
                    telemetry.update();
                }
            }
        if (detections != null) {
            telemetry.addData("detections size", detections.size());
            telemetry.update();
        }else{
            telemetry.addData("detections is null", "yes");
            telemetry.update();
        }
            sleep(20);
        return true;}


    public void resetArm(){
        robot.shoulderStar.setPosition(shoulderStarScore-0.04);
        robot.shoulderPort.setPosition(shoulderPortScore+0.04);
        robot.elbow.setPosition(elbowNorminal);
        sleep(500);
        robot.extendyBoi.setPosition(extendyBoiRetract);
        sleep(500);
        robot.wrist.setPosition(wristDown);
        robot.door.setPosition(doorClosed);
        robot.portArm.setPower(-0.5);
        robot.starArm.setPower(-0.5);
        sleep(800);
        robot.shoulderStar.setPosition(shoulderStarDown);
        robot.shoulderPort.setPosition(shoulderPortDown);
        sleep(1000);
        robot.portArm.setPower(0);
        robot.starArm.setPower(0);

    }
    public void scorePort(){
        robot.wrist.setPosition(wristPort);
        robot.elbow.setPosition(elbowPort);
        robot.shoulderStar.setPosition(shoulderStarScore);
        robot.shoulderPort.setPosition(shoulderPortScore);
        sleep(100);
        robot.extendyBoi.setPosition(v3Hardware.extendyBoiExtend);
    }
    public void scoreStar(){
        robot.wrist.setPosition(v3Hardware.wristStar);
        robot.elbow.setPosition(v3Hardware.elbowStar);
        robot.shoulderStar.setPosition(v3Hardware.shoulderStarScore);
        robot.shoulderPort.setPosition(v3Hardware.shoulderPortScore);
        sleep(100);
        robot.extendyBoi.setPosition(v3Hardware.extendyBoiExtend);
    }
    public void scoreBack(){
        robot.shoulderStar.setPosition(v3Hardware.shoulderStarScore);
        robot.shoulderPort.setPosition(v3Hardware.shoulderPortScore);
        robot.elbow.setPosition(v3Hardware.elbowNorminal);
        robot.wrist.setPosition(v3Hardware.wristDown);
    }
}
