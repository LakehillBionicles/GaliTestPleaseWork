package org.firstinspires.ftc.teamcode.GaliV3.v3Auto;

import static org.firstinspires.ftc.teamcode.GaliV3.v3Hardware.doorClosed;
import static org.firstinspires.ftc.teamcode.GaliV3.v3Hardware.elbowNorminal;
import static org.firstinspires.ftc.teamcode.GaliV3.v3Hardware.elbowPort;
import static org.firstinspires.ftc.teamcode.GaliV3.v3Hardware.extendyBoiRetract;
import static org.firstinspires.ftc.teamcode.GaliV3.v3Hardware.shoulderPortDown;
import static org.firstinspires.ftc.teamcode.GaliV3.v3Hardware.shoulderPortLift;
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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.GaliV3.v3Hardware;
import org.firstinspires.ftc.teamcode.Vision.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.Vision.BlueColorProcessor;
import org.firstinspires.ftc.teamcode.Vision.RedColorProcessor;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.Objects;

public class v3autoBase extends LinearOpMode {
    public v3Hardware robot = new v3Hardware();
    public org.firstinspires.ftc.teamcode.Vision.RedColorProcessor RedColorProcessor;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    public org.firstinspires.ftc.teamcode.Vision.BlueColorProcessor BlueColorProcessor;
    public String  propPos = "notSeen";

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
    public void runOpMode(){
        robot.init(hardwareMap);
        robot.door.setPosition(v3Hardware.doorClosed);
        robot.extendyBoi.setPosition(v3Hardware.extendyBoiRetract);
        robot.elbow.setPosition(v3Hardware.elbowNorminal);
        robot.shoulderStar.setPosition(v3Hardware.shoulderStarDown);
        robot.shoulderPort.setPosition(v3Hardware.shoulderPortDown);
        robot.wrist.setPosition(v3Hardware.wristDown);
    }
    public void cameraStartup(String cameraName){
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
            camera.setPipeline(BlueColorProcessor);
        }
        if(Objects.equals(color, "red")){
            RedColorProcessor = new RedColorProcessor();
            camera.setPipeline(RedColorProcessor);
        }
    }
    public String propPos(String color, String placement){

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
                if(rightRedRatio>redTolerance && leftRedRatio> centerRedRatio){
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
