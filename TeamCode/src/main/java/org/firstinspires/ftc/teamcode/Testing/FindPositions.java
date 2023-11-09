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

    public static double elbowTarget = 0;
    public static double wristTarget = 0;
    public static double fingerPortTarget = 0;
    public static double fingerStarTarget = 0;
    public static int aimerTarget = 0;
    public static double triggerTarget = 0;
    public static double intakePower = 0;

    //public static double heightOfLauncher = 0;

    //public static double onOrOff = 0;

    //public static double aimerDown = 0, triggerUp = 1,aimerUp = 0.67, triggerDown = 0;


    @Override
    public void init(){
        robot.init(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot.portArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.starArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.aimer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public void loop(){
        //robot.launcherExtender.setPosition(heightOfLauncher);
        //robot.launcherStopper.setPosition(onOrOff);

        robot.aimer.setTargetPosition(aimerTarget);
        robot.aimer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.aimer.setPower(.25);

        robot.trigger.setPosition(triggerTarget);

        //robot.elbow.setPosition(elbowTarget);
        //robot.wrist.setPosition(wristTarget);
        //robot.fingerPort.setPosition(fingerPortTarget);
        //robot.fingerStar.setPosition(fingerStarTarget);

        //robot.intake.setPower(intakePower);
    }
}
