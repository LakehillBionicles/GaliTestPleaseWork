package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.java_websocket.framing.ContinuousFrame;

public class GaliHardware extends LinearOpMode {
    public DcMotor fpd = null, bpd = null, fsd = null, bsd = null, handStar = null, wrist = null;
    public DcMotor elbow = null, shoulder = null;
    public Servo aimer = null, trigger = null, fingerStar = null, fingerPort = null;
    public CRServo handPort = null;

    public static double fingerPortOpen = 0, fingerPortClosed = .15;
    public static double fingerStarOpen = 0.3, fingerStarClosed = .15;

    public static double wristUpPort = 0, wristScoreLowPort = 0.4, wristScoreHighPort = 0.5, wristPickupPort = .68;//TODO: find wrist positions
    public static double wristUpStar = 1-wristUpPort, wristScoreLowStar = 1-wristScoreLowPort, wristScoreHighStar = 1-wristScoreHighPort, wristPickupStar = 1-wristPickupPort;//TODO: find wrist positions

    public static double aimerDown = 0, triggerUp = 1, aimerUp = 0.5, triggerDown = 0;

    public static int wristUp = 0;
    public static int wristPickup = -1600;

    public static int wristScore = 0;

    HardwareMap hwMap = null;

    public GaliHardware() {}

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }
    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;

        fpd = hwMap.get(DcMotor.class, "fpd" );
        bpd = hwMap.get(DcMotor.class, "bpd");
        fsd = hwMap.get(DcMotor.class, "fsd");
        bsd = hwMap.get(DcMotor.class, "bsd");

        elbow = hwMap.get(DcMotor.class, "elbow");
        shoulder = hwMap.get(DcMotor.class,"shoulder");

        handPort = hwMap.crservo.get("handPort");
        wrist = hwMap.get(DcMotor.class, "wrist");
        handStar = hwMap.get(DcMotor.class, "handStar");

        fingerStar = hwMap.get(Servo.class, "fingerStar");
        fingerPort = hwMap.get(Servo.class, "fingerPort");

        aimer = hwMap.get(Servo.class, "aimer");
        trigger = hwMap.get(Servo.class, "trigger");

        fpd.setDirection(DcMotorSimple.Direction.FORWARD);
        bpd.setDirection(DcMotorSimple.Direction.FORWARD);
        fsd.setDirection(DcMotorSimple.Direction.FORWARD);
        bsd.setDirection(DcMotorSimple.Direction.FORWARD);

        elbow.setDirection(DcMotorSimple.Direction.REVERSE);
        shoulder.setDirection(DcMotorSimple.Direction.REVERSE);
        wrist.setDirection(DcMotorSimple.Direction.FORWARD);

        handPort.setDirection(CRServo.Direction.FORWARD);
        handStar.setDirection(DcMotorSimple.Direction.REVERSE);

        fpd.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bpd.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fsd.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bsd.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        handStar.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wrist.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        fpd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bpd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fsd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bsd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wrist.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        elbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shoulder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        handStar.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wrist.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}
