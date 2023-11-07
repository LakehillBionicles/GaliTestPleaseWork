package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.GaliHardware;

@Config
@TeleOp
public class IntakePrototype extends OpMode {
    //GaliHardware robot = new GaliHardware();
    public static double power = 0;
    public DcMotor intake;

    @Override
    public void init(){
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    @Override
    public void loop(){
        intake.setPower(power);
    }
}
