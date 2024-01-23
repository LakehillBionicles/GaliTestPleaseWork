package org.firstinspires.ftc.teamcode.GaliV3.v3Tele;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import static org.firstinspires.ftc.teamcode.GaliV3.v3Hardware.doorClosed;
import static org.firstinspires.ftc.teamcode.GaliV3.v3Hardware.doorOpen;
import static org.firstinspires.ftc.teamcode.GaliV3.v3Hardware.elbowNorminal;
import static org.firstinspires.ftc.teamcode.GaliV3.v3Hardware.elbowPort;
import static org.firstinspires.ftc.teamcode.GaliV3.v3Hardware.elbowStar;
import static org.firstinspires.ftc.teamcode.GaliV3.v3Hardware.extendyBoiExtend;
import static org.firstinspires.ftc.teamcode.GaliV3.v3Hardware.extendyBoiRetract;
import static org.firstinspires.ftc.teamcode.GaliV3.v3Hardware.shoulderPortDown;
import static org.firstinspires.ftc.teamcode.GaliV3.v3Hardware.shoulderPortLift;
import static org.firstinspires.ftc.teamcode.GaliV3.v3Hardware.shoulderPortScore;
import static org.firstinspires.ftc.teamcode.GaliV3.v3Hardware.shoulderStarDown;
import static org.firstinspires.ftc.teamcode.GaliV3.v3Hardware.shoulderStarScore;
import static org.firstinspires.ftc.teamcode.GaliV3.v3Hardware.wristDown;
import static org.firstinspires.ftc.teamcode.GaliV3.v3Hardware.wristPort;
import static org.firstinspires.ftc.teamcode.GaliV3.v3Hardware.wristStar;

import org.firstinspires.ftc.teamcode.GaliV3.v3Hardware;
@TeleOp

public class v3Tele extends teleBase {
    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();
    public double drivePower = 1;
    boolean intakeOn = false, intakeSpit = false;
    public static double intakePower = -0.8;
    double doorTimer = -2;
    double extendyBoiTimer = -2;
    String doorPos = "closed";

    @Override
    public void runOpMode(){
        robot.init(hardwareMap);
        robot.door.setPosition(v3Hardware.doorClosed);
        robot.extendyBoi.setPosition(v3Hardware.extendyBoiRetract);
        robot.elbow.setPosition(v3Hardware.elbowNorminal);
        robot.shoulderStar.setPosition(v3Hardware.shoulderStarDown);
        robot.shoulderPort.setPosition(v3Hardware.shoulderPortDown);
        robot.wrist.setPosition(v3Hardware.wristDown);
        waitForStart();
        while (opModeIsActive()) {
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);
            intakeSpit = gamepad1.x;
            if (!previousGamepad1.back && gamepad1.back) {
                intakeOn = !intakeOn;
            }
            robot.fpd.setPower((-gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x) * drivePower);
            robot.bpd.setPower((-gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x) * drivePower);
            robot.fsd.setPower((-gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x) * drivePower);
            robot.bsd.setPower((-gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x) * drivePower);
            if (intakeSpit) {
                robot.intake.setPower(-intakePower);
            } else if (intakeOn) {
                robot.intake.setPower(intakePower);
            }
            else{
                robot.intake.setPower(0);
            }
            robot.portArm.setPower(-gamepad2.left_stick_y);
            robot.starArm.setPower(-gamepad2.left_stick_y);
            if(gamepad2.dpad_up){
            robot.wrist.setPosition(wristDown);
            robot.elbow.setPosition(elbowNorminal);
            robot.shoulderStar.setPosition(shoulderStarScore);
            robot.shoulderPort.setPosition(shoulderPortScore);
            robot.extendyBoi.setPosition(extendyBoiRetract);
            }
            if(gamepad2.dpad_left){
                robot.wrist.setPosition(wristPort);
                robot.elbow.setPosition(elbowPort);
                robot.shoulderStar.setPosition(shoulderStarScore);
                robot.shoulderPort.setPosition(shoulderPortScore);
                extendyBoiTimer = getRuntime();
            }
            if(gamepad2.dpad_right){
                robot.wrist.setPosition(wristStar);
                robot.elbow.setPosition(elbowStar);
                robot.shoulderStar.setPosition(shoulderStarScore);
                robot.shoulderPort.setPosition(shoulderPortScore);
                extendyBoiTimer = getRuntime();
            }
            if(gamepad2.dpad_down){
                robot.wrist.setPosition(wristDown);
                robot.elbow.setPosition(elbowNorminal);
                robot.shoulderStar.setPosition(shoulderStarDown);
                robot.shoulderPort.setPosition(shoulderPortDown);
                robot.extendyBoi.setPosition(extendyBoiRetract);
            }
            if(extendyBoiTimer +0.3<=getRuntime()&& extendyBoiTimer +1>=getRuntime()){
                robot.extendyBoi.setPosition(extendyBoiExtend);
            }
            if(gamepad2.b){
                robot.door.setPosition(doorClosed);
                doorPos = "closed";
            }
            if(gamepad2.y){
                robot.shoulderPort.setPosition(shoulderPortLift);
                robot.shoulderStar.setPosition(v3Hardware.shoulderStarLift);
                extendyBoiTimer = getRuntime();
                robot.wrist.setPosition(v3Hardware.wristLift);
            }
            if(robot.handTS.isPressed()|| gamepad2.a){
                robot.door.setPosition(doorOpen);
                doorTimer = getRuntime();
                doorPos = "open";
            }
            else if(doorTimer+2>getRuntime()&&robot.shoulderPort.getPosition()!=shoulderPortDown&&robot.shoulderStar.getPosition()!=shoulderStarDown&&doorPos.equals("open")) {
                robot.door.setPosition(doorOpen);
            }else{
                robot.door.setPosition(doorClosed);
                doorPos = "closed";
            }
            if(gamepad1.dpad_up){
                robot.aimer.setPower(0.3);}
            if(gamepad1.dpad_down){
                robot.aimer.setPower(-0.3);}
            if(gamepad1.dpad_right){
                robot.aimer.setPower(0);}
            if(gamepad1.right_bumper&& gamepad1.left_bumper){
                robot.trigger.setPosition(1);
            }
            if(gamepad1.left_trigger>0){
                robot.trigger.setPosition(0);
            }
        }
    }
}