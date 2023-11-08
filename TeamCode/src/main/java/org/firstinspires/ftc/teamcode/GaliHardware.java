package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class GaliHardware extends LinearOpMode {
    public DcMotor fpd = null, bpd = null, fsd = null, bsd = null, SOW = null, intake = null;
    public DcMotor portArm = null, starArm = null;
    public Servo elbow = null, wrist = null, aimer = null, trigger = null, fingerStar = null, fingerPort = null;

    public static double fingerPortOpen = 0.4, fingerPortClosed = 0;
    public static double fingerStarOpen = 0.6, fingerStarClosed = 1;

    public static double wristDown = 0.45, wristScore = 0.85;

    public static double elbowDown = 0.07, elbowScore = 0.33;

    public static double aimerDown = 0, triggerUp = 1, aimerUp = 0.5, triggerDown = 0;

    HardwareMap hwMap = null;

    public GaliHardware() {}

    @Override
    public void runOpMode() {}

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;

        fpd = hwMap.get(DcMotor.class, "fpd" );
        bpd = hwMap.get(DcMotor.class, "bpd");
        fsd = hwMap.get(DcMotor.class, "fsd");
        bsd = hwMap.get(DcMotor.class, "bsd");
        SOW = hwMap.get(DcMotor.class, "SOW");

        portArm = hwMap.get(DcMotor.class, "portArm");
        starArm = hwMap.get(DcMotor.class,"starArm");

        intake = hwMap.get(DcMotor.class, "intake");

        elbow = hwMap.get(Servo.class, "elbow");
        wrist = hwMap.get(Servo.class, "wrist");
        fingerStar = hwMap.get(Servo.class, "fingerStar");
        fingerPort = hwMap.get(Servo.class, "fingerPort");

        aimer = hwMap.get(Servo.class, "aimer");
        trigger = hwMap.get(Servo.class, "trigger");

        fpd.setDirection(DcMotorSimple.Direction.REVERSE);
        bpd.setDirection(DcMotorSimple.Direction.FORWARD);
        fsd.setDirection(DcMotorSimple.Direction.REVERSE);
        bsd.setDirection(DcMotorSimple.Direction.REVERSE);

        portArm.setDirection(DcMotorSimple.Direction.FORWARD);
        starArm.setDirection(DcMotorSimple.Direction.REVERSE);

        intake.setDirection(DcMotorSimple.Direction.FORWARD);

        fpd.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bpd.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fsd.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bsd.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        portArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        starArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        fpd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bpd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fsd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bsd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        portArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        starArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}