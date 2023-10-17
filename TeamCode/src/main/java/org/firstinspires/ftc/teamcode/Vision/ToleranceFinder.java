package org.firstinspires.ftc.teamcode.Vision;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Vision.ProcessorForToleranceFinder;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
public class ToleranceFinder extends LinearOpMode{
    private ProcessorForToleranceFinder ProcessorForToleranceFinder;
    private OpenCvCamera camera;

    // Name of the Webcam to be set in the config
    private String webcamName = "Webcam 1";

    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);
        ProcessorForToleranceFinder = new ProcessorForToleranceFinder();
        camera.setPipeline(ProcessorForToleranceFinder);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });

        while (!isStarted()) {
            telemetry.addData("BlueColor", org.firstinspires.ftc.teamcode.Vision.ProcessorForToleranceFinder.getBlue());
            telemetry.addData("RedColor: ", org.firstinspires.ftc.teamcode.Vision.ProcessorForToleranceFinder.getRed());
            telemetry.addData("GreenColor: ", org.firstinspires.ftc.teamcode.Vision.ProcessorForToleranceFinder.getGreen());
            telemetry.update();
        }

        waitForStart();
        while(opModeIsActive()){
            telemetry.addData("BlueColor: ", org.firstinspires.ftc.teamcode.Vision.ProcessorForToleranceFinder.getBlue());
            telemetry.addData("RedColor: ", org.firstinspires.ftc.teamcode.Vision.ProcessorForToleranceFinder.getRed());
            telemetry.addData("GreenColor: ", org.firstinspires.ftc.teamcode.Vision.ProcessorForToleranceFinder.getGreen());
            telemetry.update();

        }
    }
}
