package org.firstinspires.ftc.teamcode.GaliAuto;

import org.openftc.apriltag.AprilTagDetection;

import java.util.ArrayList;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.GaliV3.v3Roadrunner.drive.SampleMecanumDrive;

@Autonomous

public class aprilTagAlignmentTest extends GaliAutobase{
    Pose2d startPose = new Pose2d(0, 0, 0);
    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        cameraStartup("Webcam 1");
        aprilTagDetection();
        int sideOfSleeve= 3;
        boolean stayInLoop = true;
        while(!opModeIsActive()&& stayInLoop){
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
                            sideOfSleeve = detection.id;
                            telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
                            telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
                            telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
                            telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", detection.pose.R));
                            if(sideOfSleeve == 3){
                                stayInLoop = false;
                            }
                        }
                    }

                    telemetry.update();
                }


                sleep(20);
            }

        }
        waitForStart();
        while (opModeIsActive()){

        }
        }
    }
