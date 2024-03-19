package org.firstinspires.ftc.teamcode.GaliAuto;


import static org.firstinspires.ftc.teamcode.GaliHardware.elbowDown;
import static org.firstinspires.ftc.teamcode.GaliHardware.elbowLift;
import static org.firstinspires.ftc.teamcode.GaliHardware.elbowScore;
import static org.firstinspires.ftc.teamcode.GaliHardware.fingerPortClosed;
import static org.firstinspires.ftc.teamcode.GaliHardware.fingerPortOpen;
import static org.firstinspires.ftc.teamcode.GaliHardware.fingerStarClosed;
import static org.firstinspires.ftc.teamcode.GaliHardware.fingerStarOpen;
import static org.firstinspires.ftc.teamcode.GaliHardware.wristDown;
import static org.firstinspires.ftc.teamcode.GaliHardware.wristLift;
import static org.firstinspires.ftc.teamcode.GaliHardware.wristScore;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.GaliHardware;
import org.firstinspires.ftc.teamcode.Vision.AprilTagDetectionPipeline2;
import org.firstinspires.ftc.teamcode.Vision.BlueColorProcessor;
import org.firstinspires.ftc.teamcode.Vision.RedColorProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.Objects;

public class GaliAutobase extends LinearOpMode {
    public GaliHardware robot = new GaliHardware();
    public RedColorProcessor RedColorProcessor;
    AprilTagDetectionPipeline2 aprilTagDetectionPipeline;
    public BlueColorProcessor BlueColorProcessor;
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
    @Override
    public void runOpMode(){
        robot.init(hardwareMap);
    }

    public void armLift(){
        robot.elbow.setPosition(elbowLift);
        robot.wrist.setPosition(wristLift);
    }
    public void armDown(){
        robot.elbow.setPosition(elbowDown);
        robot.wrist.setPosition(wristDown);
    }
    public void armUp(){
        robot.wrist.setPosition(wristScore);
        robot.elbow.setPosition(elbowScore);
    }
    public void fingersClosed(){
        robot.fingerStar.setPosition(fingerStarClosed);
        robot.fingerPort.setPosition(fingerPortClosed);
    }
    public void fingersOpened(){
        robot.fingerStar.setPosition(fingerStarOpen);
        robot.fingerPort.setPosition(fingerPortOpen);
    }
    public void aprilTagAlignment(){
        resetCamera();
        aprilTagDetection();
    }
    public void aprilTagDetection(){
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline2(tagsize, fx, fy, cx, cy);
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
        /*
        while (!opModeIsActive()){
            telemetry.addData("isItInOpMode","yes");
            telemetry.update();
            // Calling getDetectionsUpdate() will only return an object if there was a new frame
            // processed since the last time we called it. Otherwise, it will return null. This
            // enables us to only run logic when there has been a new frame, as opposed to the
            // getLatestDetections() method which will always return an object.
            ArrayList<org.openftc.apriltag.AprilTagDetection> detections = aprilTagDetectionPipeline.getDetectionsUpdate();
            // If there's been a new frame...
            if(detections != null)
            {
                telemetry.addData("FPS", camera.getFps());
                telemetry.addData("Overhead ms", camera.getOverheadTimeMs());
                telemetry.addData("Pipeline ms", camera.getPipelineTimeMs());

                // If we don't see any tags
                if(detections.size() == 0)
                {
                    numFramesWithoutDetection++;

                    // If we haven't seen a tag for a few frames, lower the decimation
                    // so we can hopefully pick one up if we're e.g. far back
                    if(numFramesWithoutDetection >= THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION)
                    {
                        aprilTagDetectionPipeline.setDecimation(DECIMATION_LOW);
                    }
                }
                // We do see tags!
                else
                {
                    numFramesWithoutDetection = 0;

                    // If the target is within 1 meter, turn on high decimation to
                    // increase the frame rate
                    if(detections.get(0).pose.z < THRESHOLD_HIGH_DECIMATION_RANGE_METERS)
                    {
                        aprilTagDetectionPipeline.setDecimation(DECIMATION_HIGH);
                    }

                    for(AprilTagDetection detection : detections)
                    {
                        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
                        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
                        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
                        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", detection.pose.R));
                    }
                }

                telemetry.update();
            }


            sleep(20);
        }

         */
    }
    public void resetCamera(){
        camera.stopStreaming();
        camera.closeCameraDeviceAsync(() -> {});
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
            propPos = org.firstinspires.ftc.teamcode.Vision.BlueColorProcessor.pos;
        }
        if(Objects.equals(color, "red")){
            RedColorProcessor = new RedColorProcessor();
            camera.setPipeline(RedColorProcessor);
            propPos = org.firstinspires.ftc.teamcode.Vision.RedColorProcessor.pos;
        }
    }

}
