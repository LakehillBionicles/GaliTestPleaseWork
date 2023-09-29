package org.firstinspires.ftc.teamcode.teamcode;
/*
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
 */
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

//@Config
@TeleOp
public class FindPositions extends OpMode {
    GaliHardware robot = new GaliHardware();

    public static int elbowTarget = 0;
    public static int shoulderTarget = 0;
    public static double wristTarget = 1;
    public static double handPortTarget = 0.6;
    public static double handStarTarget = 0.1;

    @Override
    public void init(){
        robot.init(hardwareMap);
        //telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot.elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public void loop(){
        robot.elbow.setTargetPosition(elbowTarget);
        robot.elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.elbow.setPower(.25);

        robot.shoulder.setTargetPosition(shoulderTarget);
        robot.shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.shoulder.setPower(.25);

        //robot.wrist.setPosition(wristTarget);

        robot.handPort.setPosition(handPortTarget);
        robot.handStar.setPosition(handStarTarget);

        telemetry.addData("elbowPos", robot.elbow.getCurrentPosition());
        telemetry.addData("shoulderPos", robot.shoulder.getCurrentPosition());
        telemetry.update();
    }
}
