package org.firstinspires.ftc.teamcode.GaliV3.v3Tele;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.GaliV3.v3Hardware;
import static org.firstinspires.ftc.teamcode.GaliV3.v3Hardware.doorClosed;
import static org.firstinspires.ftc.teamcode.GaliV3.v3Hardware.doorOpen;
import static org.firstinspires.ftc.teamcode.GaliV3.v3Hardware.elbowNorminal;
import static org.firstinspires.ftc.teamcode.GaliV3.v3Hardware.elbowPort;
import static org.firstinspires.ftc.teamcode.GaliV3.v3Hardware.elbowStar;
import static org.firstinspires.ftc.teamcode.GaliV3.v3Hardware.extendyBoiDown;
import static org.firstinspires.ftc.teamcode.GaliV3.v3Hardware.extendyBoiExtend;
import static org.firstinspires.ftc.teamcode.GaliV3.v3Hardware.extendyBoiRetract;
import static org.firstinspires.ftc.teamcode.GaliV3.v3Hardware.shoulderPortDown;
import static org.firstinspires.ftc.teamcode.GaliV3.v3Hardware.shoulderPortLift;
import static org.firstinspires.ftc.teamcode.GaliV3.v3Hardware.shoulderPortScore;
import static org.firstinspires.ftc.teamcode.GaliV3.v3Hardware.shoulderStarDown;
import static org.firstinspires.ftc.teamcode.GaliV3.v3Hardware.shoulderStarScore;
import static org.firstinspires.ftc.teamcode.GaliV3.v3Hardware.wristDown;
import static org.firstinspires.ftc.teamcode.GaliV3.v3Hardware.wristLift;
import static org.firstinspires.ftc.teamcode.GaliV3.v3Hardware.wristPort;
import static org.firstinspires.ftc.teamcode.GaliV3.v3Hardware.wristStar;

import org.firstinspires.ftc.teamcode.GaliV3.v3Hardware;
@Config
@TeleOp
public class findShoulderPosition extends LinearOpMode {
    public v3Hardware robot = new v3Hardware();
    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();
    public double drivePower = 1;
    boolean intakeOn = false, intakeSpit = false;
    double doorTimer = -2;
    double extendyBoiTimerExtend = -2;
    double extendyBoiTimeRetract = -2;
    double wristTimerDown = -2;

    double shoulderTimerDown = -2;

    double armDownTimer = -2;

    String doorPos = "closed";
    double elbowStarTimer = -3;
    double elbowPortTimer = -3;
    double wristLiftTimer =-3;
    String armPos = "down";
    public static double shoulderPos = 0.5;
    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        //robot.door.setPosition(v3Hardware.doorClosed);
        //robot.extendyBoi.setPosition(v3Hardware.extendyBoiDown);
        //robot.elbow.setPosition(v3Hardware.elbowNorminal);
        //robot.shoulderStar.setPosition(v3Hardware.shoulderStarDown);
        //robot.shoulderStar.setPosition(shoulderPos);
        //robot.wrist.setPosition(v3Hardware.wristDown);
        waitForStart();
        while (opModeIsActive()) {
            //robot.shoulderStar.setPosition(shoulderPos);
            robot.aimer.setPower(1);













        }
    }
}
