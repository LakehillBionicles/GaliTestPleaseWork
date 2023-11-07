package org.firstinspires.ftc.teamcode.Testing;
/*
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;

 */
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.GaliHardware;

//@Disabled
//@Config
@TeleOp
public class ShoulderPIDTuning extends OpMode {
    GaliHardware robot = new GaliHardware();
    //private PIDController controller;

    public static double p = 0.0, i = 0.0, d = 0.0;
    public static int reference = 0;

    @Override
    public void init(){
        robot.init(hardwareMap);
        //controller = new PIDController(p, i, d);
       // telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //robot.shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //robot.shoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //robot.wrist.setPosition(1);
    }

    @Override
    public void loop(){
        //controller.setPID(p, i, d);

        //double state = robot.shoulder.getCurrentPosition();
        //double pid = controller.calculate(state, reference);

        //robot.shoulder.setPower(pid);

        telemetry.addData("reference", reference);
        //telemetry.addData("state", state);
        //telemetry.addData("power", pid);
        telemetry.update();
    }
}