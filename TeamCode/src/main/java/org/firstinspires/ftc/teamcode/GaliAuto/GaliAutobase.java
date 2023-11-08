package org.firstinspires.ftc.teamcode.GaliAuto;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.GaliHardware;
import org.firstinspires.ftc.teamcode.Vision.BlueColorProcessor;
import org.firstinspires.ftc.teamcode.Vision.RedColorProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.Objects;

public class GaliAutobase extends LinearOpMode {
    public GaliHardware robot = new GaliHardware();
    public RedColorProcessor RedColorProcessor;
    public BlueColorProcessor BlueColorProcessor;
    public String  propPos = "notSeen";

    public BNO055IMU imu;
    private String webcam1 = "Webcam 1";
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;
    double tagsize = 0.166;
    OpenCvCamera camera;
    @Override
    public void runOpMode(){
        robot.init(hardwareMap);
    }
    public void propDetection(String color){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcam1), cameraMonitorViewId);
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
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode) {
            }
        });

    }

}
