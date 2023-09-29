package org.firstinspires.ftc.teamcode.teamcode.Testing;

//import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.teamcode.GaliHardware;

//@Config
@TeleOp
public class IntakePrototype extends OpMode {
    GaliHardware robot = new GaliHardware();
    public static double power = 0;

    @Override
    public void init(){
        robot.init(hardwareMap);
        robot.BOW.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop(){
        robot.POW.setPower(power);
        robot.BOW.setPower(power);
    }
}
