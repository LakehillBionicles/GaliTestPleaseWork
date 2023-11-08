package org.firstinspires.ftc.teamcode.Tele;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.apache.commons.math3.geometry.euclidean.twod.Line;
@TeleOp
public class motorTest extends LinearOpMode {
    public DcMotor motor1 = null;
    HardwareMap hwMap = null;

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;

        motor1 = hwMap.get(DcMotor.class, "motor1");
    }

    @Override
    public void runOpMode() {
        init(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            motor1.setPower(gamepad1.left_stick_y);
        }

    }
}
