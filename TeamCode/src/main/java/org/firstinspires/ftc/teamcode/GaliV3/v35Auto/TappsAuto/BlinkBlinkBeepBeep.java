package org.firstinspires.ftc.teamcode.GaliV3.v35Auto.TappsAuto;

import static org.firstinspires.ftc.teamcode.Vision.BlueColorProcessor.centerBlueRatio;
import static org.firstinspires.ftc.teamcode.Vision.BlueColorProcessor.leftBlueRatio;
import static org.firstinspires.ftc.teamcode.Vision.BlueColorProcessor.rightBlueRatio;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.GaliV3.v3Auto.v3autoBase;
import org.firstinspires.ftc.teamcode.GaliV3.v3Hardware;
import org.firstinspires.ftc.teamcode.GaliV3.v3Roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.GaliV3.v3Roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Vision.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
@Autonomous
public class BlinkBlinkBeepBeep extends v3autoBase {
    public v3Hardware robot = new v3Hardware();
    public org.firstinspires.ftc.teamcode.Vision.RedColorProcessor RedColorProcessor;
    public org.firstinspires.ftc.teamcode.Vision.BlueColorProcessor BlueColorProcessor;
    public String  propPos = "notSeen";
    public static String pipeline = "";
    public Orientation robotOrientation = new Orientation();
    public static String robotPosition = "notSeen";
    public double x = 0;
    public double y = 0;
    public double z = 0;
    public double yaw = 0;
    private String webcam = "Webcam ";
    public Orientation robotTheta;
    public IMU imu;
    public YawPitchRollAngles yawPitchRollAngles = new YawPitchRollAngles(AngleUnit.DEGREES, 0, 0,0, (long) 2);
    static final double FEET_PER_METER = 3.28084;
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;
    double tagsize = 0.05;
    Pose2d startPose = new Pose2d(-38, 61, Math.toRadians(-90));
    public static double forWard = 0;
    public static double turn1 = 0;
    AprilTagDetectionPipeline aprilTagDetectionPipeline2= new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
    int numFramesWithoutDetection = 0;

    final float DECIMATION_HIGH = 3;
    final float DECIMATION_LOW = 2;
    final float THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f;
    final int THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION = 4;
    OpenCvCamera camera;
    OpenCvCamera camera1;
    boolean stayInLoop = true;
    double headingError = 0;
    double i = 0;
    double blinkerGatekeep = 0;
    @Override
    public void runOpMode() {
        super.runOpMode();
        robot.init(hardwareMap);
        imu = hardwareMap.get(IMU.class, "imu");
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
       /* while (!isStarted()){
            /*telemetry.addData("blinker left", robot.blinkerPort.getDistance(DistanceUnit.MM));
            telemetry.addData("blinker right", robot.blinkerStar.getDistance(DistanceUnit.MM));
            telemetry.update();


        }
        */
        waitForStart();
        if (opModeIsActive()){
            int i=0;
            drive.setPoseEstimate(new Pose2d(0 , 0, 0));
            while(robot.blinkerStar.getDistance(DistanceUnit.MM) <240 && robot.blinkerPort.getDistance(DistanceUnit.MM) <240 && i<12 ){
                drive.strafeRight(4);
                drive.turn(Math.toRadians(drive.getPoseEstimate().getHeading()));
                i++;
                telemetry.addData("heading",drive.getPoseEstimate().getHeading());
                telemetry.addData("blinker left", robot.blinkerPort.getDistance(DistanceUnit.MM));
                telemetry.addData("blinker right", robot.blinkerStar.getDistance(DistanceUnit.MM));
                telemetry.update();
            }
            i=0;
            while(robot.blinkerStar.getDistance(DistanceUnit.MM) >240 && robot.blinkerPort.getDistance(DistanceUnit.MM) <240 && i<12 ){
                drive.strafeLeft(1);
                drive.turn(-Math.toRadians(-drive.getPoseEstimate().getHeading()));
                i++;
                telemetry.addData("heading",drive.getPoseEstimate().getHeading());
                telemetry.addData("blinker left", robot.blinkerPort.getDistance(DistanceUnit.MM));
                telemetry.addData("blinker right", robot.blinkerStar.getDistance(DistanceUnit.MM));
                telemetry.update();
            }
            drive.turn(-Math.toRadians(-drive.getPoseEstimate().getHeading()));
            robot.shoulderStar.setPosition(v3Hardware.shoulderStarScore);
            robot.shoulderPort.setPosition(v3Hardware.shoulderPortScore);
            sleep(10000);
        }
    }
}