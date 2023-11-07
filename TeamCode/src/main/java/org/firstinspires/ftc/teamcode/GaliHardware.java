package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class GaliHardware extends LinearOpMode {
    public DcMotor fpd = null, bpd = null, fsd = null, bsd = null, SOW = null, intake = null;
    public DcMotor armPort = null, armStar = null;
    public Servo wrist = null, aimer = null, trigger = null, fingerStar = null, fingerPort = null;

    public static double fingerPortOpen = 0, fingerPortClosed = .15;
    public static double fingerStarOpen = 0.3, fingerStarClosed = .15;

    public static double wristUpPort = 0, wristScoreLowPort = 0.4, wristScoreHighPort = 0.5, wristPickupPort = .68;
    public static double wristUpStar = 1-wristUpPort, wristScoreLowStar = 1-wristScoreLowPort, wristScoreHighStar = 1-wristScoreHighPort, wristPickupStar = 1-wristPickupPort;//TODO: find wrist positions

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

        armPort = hwMap.get(DcMotor.class, "armPort");
        armStar = hwMap.get(DcMotor.class,"armStar");

        intake = hwMap.get(DcMotor.class, "intake");

        wrist = hwMap.get(Servo.class, "wrist");
        fingerStar = hwMap.get(Servo.class, "fingerStar");
        fingerPort = hwMap.get(Servo.class, "fingerPort");

        aimer = hwMap.get(Servo.class, "aimer");
        trigger = hwMap.get(Servo.class, "trigger");

        fpd.setDirection(DcMotorSimple.Direction.FORWARD);
        bpd.setDirection(DcMotorSimple.Direction.FORWARD);
        fsd.setDirection(DcMotorSimple.Direction.FORWARD);
        bsd.setDirection(DcMotorSimple.Direction.FORWARD);

        armPort.setDirection(DcMotorSimple.Direction.REVERSE);
        armStar.setDirection(DcMotorSimple.Direction.REVERSE);

        intake.setDirection(DcMotorSimple.Direction.FORWARD);

        fpd.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bpd.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fsd.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bsd.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        armPort.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armStar.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        fpd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bpd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fsd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bsd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armPort.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armStar.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}
