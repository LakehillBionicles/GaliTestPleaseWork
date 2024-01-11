package org.firstinspires.ftc.teamcode.Vision;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@Config
@Disabled
@Autonomous(name = "BluePropTest")
public class VisionForDetectingPropBlue extends LinearOpMode {
    private OpenCvCamera camera;

    // Name of the Webcam to be set in the config
    private String webcamName = "Webcam 1";
    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);
        org.firstinspires.ftc.teamcode.Vision.BlueColorProcessor BlueColorProcessor = new BlueColorProcessor();
        camera.setPipeline(BlueColorProcessor);
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
            telemetry.addData("Pos: ", BlueColorProcessor.pos);
            /*
            try {
                telemetry.addData("leftTotal", blueColorProcessor.leftTotal);
            } catch (NullPointerException ignored) {
            }
            try {
                telemetry.addData("rightTotal", blueColorProcessor.rightTotal);
            } catch (NullPointerException ignored) {
            }
            try {
                telemetry.addData("centerTotal", blueColorProcessor.centerTotal);
            } catch (NullPointerException ignored) {
            }
            try {
                telemetry.update();
            } catch (NullPointerException ignored) {
            }

             */
            telemetry.addData("left", BlueColorProcessor.leftTotal);
            telemetry.addData("center", BlueColorProcessor.centerTotal);
            telemetry.addData("right", BlueColorProcessor.leftTotal);
            telemetry.update();
        }
        waitForStart();
        while(opModeIsActive()){
            telemetry.addData("Pos: ", BlueColorProcessor.pos);
            telemetry.update();
        }
    }
}