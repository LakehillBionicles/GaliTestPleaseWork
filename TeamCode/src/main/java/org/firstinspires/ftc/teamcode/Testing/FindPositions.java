package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.GaliHardware;

@Config
@TeleOp
public class FindPositions extends OpMode {
    GaliHardware robot = new GaliHardware();

    public static int elbowTarget = 0;
    public static int shoulderTarget = 0;
    public static double wristTargetPort = 0;
    public static double wristTargetStar = 1-wristTargetPort;
    public static double fingerPortTarget = 0;
    public static double fingerStarTarget = 1;
    public static double aimerTarget = 0;
    public static double triggerTarget = 0;

    public static double heightOfLauncher = 0;

    public static double onOrOff = 0;

    //public static double aimerDown = 0, triggerUp = 1,aimerUp = 0.5, triggerDown = 0;


    @Override
    public void init(){
        robot.init(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot.elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public void loop(){
        /*robot.elbow.setTargetPosition(elbowTarget);
        robot.elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.elbow.setPower(.25);

        robot.shoulder.setTargetPosition(shoulderTarget);
        robot.shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.shoulder.setPower(.25);*/

        robot.aimer.setPosition(heightOfLauncher);
        robot.trigger.setPosition(onOrOff);

        //robot.aimer.setPosition(aimerTarget);
        //robot.trigger.setPosition(triggerTarget);

        robot.wristPort.setPosition(wristTargetPort);
        robot.wristStar.setPosition(wristTargetStar);
        //robot.fingerPort.setPosition(fingerPortTarget);
        //robot.fingerStar.setPosition(fingerStarTarget);

        //robot.handPort.setPosition(handPortTarget);
        //robot.handStar.setPosition(handStarTarget);

        telemetry.addData("elbowPos", robot.elbow.getCurrentPosition());
        telemetry.addData("shoulderPos", robot.shoulder.getCurrentPosition());
        telemetry.update();
    }
}
