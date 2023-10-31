package org.firstinspires.ftc.teamcode.Testing;

//import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.GaliHardware;
import org.firstinspires.ftc.teamcode.Roadrunner.Testing.Forward;

//@Config
@TeleOp
public class IntakePrototype extends OpMode {
    GaliHardware robot = new GaliHardware();
    public static double power = 0;

    @Override
    public void init(){
        robot.init(hardwareMap);
        //robot.BOW.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop(){
        robot.elbow.setPower(1);
        robot.shoulder.setPower(0.75);

        //robot.POW.setPower(power);
        //robot.BOW.setPower(power);
    }
}
