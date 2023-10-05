package org.firstinspires.ftc.teamcode.Vision;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Vision.ConeDetection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
@Autonomous

public class propDetectionTest extends LinearOpMode{
        private ConeDetection ConeDetection;
        private OpenCvCamera camera;

        // Name of the Webcam to be set in the config
        private String webcamName = "Webcam 1";

        @Override
        public void runOpMode() throws InterruptedException {
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);
            ConeDetection = new ConeDetection();
            camera.setPipeline(ConeDetection);

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
                telemetry.addData("Color: ", org.firstinspires.ftc.teamcode.Vision.ConeDetection.getPositionOfBlueObject());
                telemetry.addData("Color: ", org.firstinspires.ftc.teamcode.Vision.ConeDetection.getPositionOfRedObject());
                telemetry.update();
            }

            waitForStart();
            while(opModeIsActive()){
                telemetry.addData("Color: ", org.firstinspires.ftc.teamcode.Vision.ConeDetection.getPositionOfBlueObject());
                telemetry.addData("Color: ", org.firstinspires.ftc.teamcode.Vision.ConeDetection.getPositionOfRedObject());
                telemetry.update();

            }
        }
}
