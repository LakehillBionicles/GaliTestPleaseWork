package org.firstinspires.ftc.teamcode.GaliV3.v3Tele;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

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
    boolean shoulderPort = false;
    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        //robot.door.setPosition(v3Hardware.doorClosed);
        //robot.extendyBoi.setPosition(v3Hardware.extendyBoiDown);
        //robot.elbow.setPosition(v3Hardware.elbowNorminal);
        //robot.shoulderStar.setPosition(v3Hardware.shoulderStarDown);
        //robot.shoulderStar.setPosition(shoulderPos);
        //robot.wrist.setPosition(v3Hardware.wristDown);
        robot.shoulderPort.close();
        robot.shoulderStar.close();
        waitForStart();
        while(!gamepad1.b||!gamepad1.x){
            telemetry.addData("press b to choose shoulderPort", "");
            telemetry.addData("press x to choose shoulderStar", "");
            telemetry.update();
        }
        if(gamepad1.b){
            robot.shoulderPort = hardwareMap.get(Servo.class, "shoulderPort");
            robot.shoulderPort.setPosition(shoulderPos);
            shoulderPort = true;
        }
        else{
            robot.shoulderStar = hardwareMap.get(Servo.class, "shoulderStar");
            robot.shoulderStar.setPosition(shoulderPos);
            shoulderPort = false;
        }
        while (opModeIsActive()) {
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);
            if(shoulderPort){
                robot.shoulderPort.setPosition(shoulderPos);
            }
            else{
                robot.shoulderStar.setPosition(shoulderPos);
            }
            if(gamepad1.b){
                robot.shoulderStar.close();
                robot.shoulderPort = hardwareMap.get(Servo.class, "portArm");
                robot.shoulderPort.setPosition(shoulderPos);
                shoulderPort = true;
            }
            else if(gamepad1.x){
                robot.shoulderPort.close();
                robot.shoulderStar = hardwareMap.get(Servo.class, "portArm");
                robot.shoulderStar.setPosition(shoulderPos);
                shoulderPort = false;
            }
            if(!previousGamepad1.dpad_up&&gamepad1.dpad_up){
                shoulderPos = shoulderPos+0.1;
            }
            if(!previousGamepad1.dpad_down&&gamepad1.dpad_down){
                shoulderPos = shoulderPos-0.1;
            }
            if(!previousGamepad1.dpad_right&&gamepad1.dpad_right){
                shoulderPos = shoulderPos+0.01;
            }
            if(!previousGamepad1.dpad_left&&gamepad1.dpad_left){
                shoulderPos = shoulderPos-0.01;
            }
            if(!previousGamepad1.y&&gamepad1.y){
                shoulderPos = shoulderPos+0.001;
            }
            if(!previousGamepad1.a&&gamepad1.a){
                shoulderPos = shoulderPos-0.001;
            }
        }
    }
}
