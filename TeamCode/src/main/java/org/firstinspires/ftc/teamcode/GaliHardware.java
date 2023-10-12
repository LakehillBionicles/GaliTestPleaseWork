package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class GaliHardware extends LinearOpMode {
    public DcMotor fpd = null, bpd = null, fsd = null, bsd = null, BOW = null, POW = null, handPort = null, handStar = null;
    public DcMotor elbow = null, shoulder = null;
    public Servo wrist = null, launcherExtender = null, launcherStopper = null, fingerStar = null, fingerPort = null;

    public double fingerPortOpen = 0, fingerPortClosed = 1;
    public double fingerStarOpen = 0, fingerStarClosed = 1;

    public double wristScore = 0, wristPickup = 0;

    public double launcherExtenderDown = 0, launcherHold = 1,launcherExtenderUp = 0.67, launcherRelease = 0;

    HardwareMap hwMap = null;

    public GaliHardware() {
    }

    @Override
    public void runOpMode() {
    }

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;

        fpd = hwMap.get(DcMotor.class, "fpd" );
        bpd = hwMap.get(DcMotor.class, "bpd");
        fsd = hwMap.get(DcMotor.class, "fsd");
        bsd = hwMap.get(DcMotor.class, "bsd");

        POW = hwMap.get(DcMotor.class, "POW");
        BOW = hwMap.get(DcMotor.class, "BOW");

        elbow = hwMap.get(DcMotor.class, "elbow");
        shoulder = hwMap.get(DcMotor.class,"shoulder");

        handPort = hwMap.get(DcMotor.class, "handPort");
        handStar = hwMap.get(DcMotor.class, "handStar");
        wrist = hwMap.get(Servo.class, "wrist");
        launcherExtender = hwMap.get(Servo.class, "launcherExtender");
        launcherStopper = hwMap.get(Servo.class, "launcherStopper");
        fingerStar = hwMap.get(Servo.class, "fingerStar");
        fingerPort = hwMap.get(Servo.class, "fingerPort");

        fpd.setDirection(DcMotorSimple.Direction.FORWARD);
        bpd.setDirection(DcMotorSimple.Direction.REVERSE);
        fsd.setDirection(DcMotorSimple.Direction.FORWARD);
        bsd.setDirection(DcMotorSimple.Direction.FORWARD);

        POW.setDirection(DcMotorSimple.Direction.FORWARD);
        BOW.setDirection(DcMotorSimple.Direction.FORWARD);

        elbow.setDirection(DcMotorSimple.Direction.REVERSE);
        shoulder.setDirection(DcMotorSimple.Direction.REVERSE);
        handPort.setDirection(DcMotorSimple.Direction.FORWARD);
        handStar.setDirection(DcMotorSimple.Direction.REVERSE);
        fpd.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bpd.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fsd.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bsd.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        POW.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BOW.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        handPort.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        handStar.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fpd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bpd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fsd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bsd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        POW.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BOW.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        elbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shoulder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        handPort.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        handStar.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }
}
