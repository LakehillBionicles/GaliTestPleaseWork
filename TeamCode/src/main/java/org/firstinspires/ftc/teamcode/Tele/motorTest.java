package org.firstinspires.ftc.teamcode.Tele;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.apache.commons.math3.geometry.euclidean.twod.Line;
@Config
@TeleOp
public class motorTest extends LinearOpMode {
    public DcMotor motor1 = null;
    public Servo fingerStar = null, fingerPort = null;
    public static double intakeOn = 0.5;
    HardwareMap hwMap = null;

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;

        motor1 = hwMap.get(DcMotor.class, "intake");
        fingerStar = hwMap.get(Servo.class, "fingerStar");
        fingerPort = hwMap.get(Servo.class, "fingerPort");
    }

    @Override
    public void runOpMode() {
        init(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            if(gamepad2.x) {
                motor1.setPower(intakeOn);
            }
            if(gamepad2.y){
                motor1.setPower(0);
            }
            if(gamepad2.a){
                fingerPort.setPosition(0);
                fingerStar.setPosition(1);
            } else if(gamepad2.b){
                fingerPort.setPosition(.4);
                fingerStar.setPosition(.6);
            }
        }

    }
}
