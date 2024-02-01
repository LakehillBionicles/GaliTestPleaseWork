package org.firstinspires.ftc.teamcode.GaliV3.v3Tele;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

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
@TeleOp

public class ONEPersonv3TeleHahaJason extends teleBase {
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
    double armUpTimer = -3;

    String doorPos = "closed";
    double elbowStarTimer = -3;
    double elbowPortTimer = -3;
    double wristLiftTimer =-3;
    String armPos = "down";

    @Override
    public void runOpMode(){
        robot.init(hardwareMap);
        robot.door.setPosition(v3Hardware.doorClosed);
        robot.extendyBoi.setPosition(v3Hardware.extendyBoiDown);
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
                robot.intake.setPower(-org.firstinspires.ftc.teamcode.GaliV3.v3Hardware.intakeSpeed);
            } else if (intakeOn) {
                robot.intake.setPower(org.firstinspires.ftc.teamcode.GaliV3.v3Hardware.intakeSpeed);
            }
            else{
                robot.intake.setPower(0);
            }
            if(gamepad1.dpad_up){
                robot.wrist.setPosition(wristDown);
                robot.elbow.setPosition(elbowNorminal);
                robot.shoulderStar.setPosition(shoulderStarScore);
                robot.shoulderPort.setPosition(shoulderPortScore);
                robot.extendyBoi.setPosition(extendyBoiRetract);
                wristTimerDown = -3;
                shoulderTimerDown = -3;
                armDownTimer = -3;
                armPos = "up";
            }
            if(gamepad1.dpad_left){
                if(robot.shoulderStar.getPosition()==shoulderStarDown||armPos.equals("down")){
                    robot.shoulderStar.setPosition(shoulderStarScore);
                    robot.shoulderPort.setPosition(shoulderPortScore);
                    robot.extendyBoi.setPosition(extendyBoiRetract);
                    wristTimerDown = -3;
                    shoulderTimerDown = -3;
                    armDownTimer = -3;
                    elbowPortTimer = getRuntime();

                }
                else if(elbowPortTimer+1.4<getRuntime()){
                    robot.wrist.setPosition(wristPort);
                    robot.elbow.setPosition(elbowPort);
                    robot.shoulderStar.setPosition(shoulderStarScore);
                    robot.shoulderPort.setPosition(shoulderPortScore);
                    extendyBoiTimerExtend = getRuntime();
                    armUpTimer = getRuntime();
                }
                armPos = "left";
            }
            if(gamepad1.dpad_right){
                if(robot.shoulderStar.getPosition()==shoulderStarDown||armPos.equals("down")){
                    robot.shoulderStar.setPosition(shoulderStarScore);
                    robot.shoulderPort.setPosition(shoulderPortScore);
                    robot.extendyBoi.setPosition(extendyBoiRetract);
                    wristTimerDown = -3;
                    shoulderTimerDown = -3;
                    armDownTimer = -3;
                    elbowStarTimer = getRuntime();
                }
                else if(elbowStarTimer+1.4<getRuntime()){
                    robot.wrist.setPosition(wristStar);
                    robot.elbow.setPosition(elbowStar);
                    robot.shoulderStar.setPosition(shoulderStarScore);
                    robot.shoulderPort.setPosition(shoulderPortScore);
                    extendyBoiTimerExtend = getRuntime();
                    armUpTimer = getRuntime();
                }
                armPos = "right";
            }
            if((elbowStarTimer+0.5<getRuntime()&&elbowStarTimer+0.7>getRuntime())||(elbowStarTimer+0.5<getRuntime()&&elbowStarTimer+0.7>getRuntime())){
                robot.portArm.setPower(0.5);
                robot.starArm.setPower(0.4);
            }
            if(elbowStarTimer + 1.4<getRuntime()&& elbowStarTimer +1.6>getRuntime()){
                robot.wrist.setPosition(wristStar);
                robot.elbow.setPosition(elbowStar);
                extendyBoiTimerExtend = getRuntime();
                robot.portArm.setPower(0);
                robot.starArm.setPower(0);
            }
            if(elbowPortTimer + 1.4<getRuntime()&& elbowPortTimer +1.6>getRuntime()){
                robot.wrist.setPosition(wristPort);
                robot.portArm.setPower(0);
                robot.portArm.setPower(0);
                robot.elbow.setPosition(elbowPort);
                extendyBoiTimerExtend = getRuntime();
            }
            telemetry.addData("elbowPos", robot.elbow.getPosition());
            if(gamepad1.dpad_down){
                if(armPos.equals("right")||armPos.equals("left")||robot.elbow.getPosition() == elbowStar|| robot.elbow.getPosition() == elbowPort||robot.wrist.getPosition()== wristPort||robot.wrist.getPosition()== wristStar){
                    robot.shoulderStar.setPosition(shoulderStarScore - 0.04);
                    robot.shoulderPort.setPosition(shoulderPortScore + 0.04);
                    robot.elbow.setPosition(elbowNorminal);
                    robot.starArm.setPower(-0.8);
                    robot.portArm.setPower(-0.8);
                    robot.extendyBoi.setPosition(extendyBoiTimeRetract);
                    wristTimerDown = getRuntime();
                    shoulderTimerDown = getRuntime();
                    armDownTimer = getRuntime();
                }
                else if(wristTimerDown+1 < getRuntime()){
                    robot.wrist.setPosition(wristDown);
                    robot.elbow.setPosition(elbowNorminal);
                    robot.shoulderStar.setPosition(shoulderStarDown);
                    robot.shoulderPort.setPosition(shoulderPortDown);
                    robot.extendyBoi.setPosition(extendyBoiDown);
                }
                armPos = "down";
            }
            if(wristTimerDown+0.5<getRuntime()&&wristTimerDown+1.2>getRuntime()){
                robot.wrist.setPosition(wristDown);
            }
            if(wristTimerDown+1<getRuntime()&&wristTimerDown+1.2>getRuntime()){
                robot.shoulderStar.setPosition(shoulderStarDown);
                robot.shoulderPort.setPosition(shoulderPortDown);
                robot.extendyBoi.setPosition(extendyBoiDown);
            }
            telemetry.addData("armdownTimer", armDownTimer);
            telemetry.addData("runtime",getRuntime());
            telemetry.update();
            if(armDownTimer+1.5< getRuntime()&&armUpTimer+0.9>getRuntime()){
                if(gamepad1.left_trigger>0){
                    robot.portArm.setPower(-gamepad1.left_trigger);
                    robot.starArm.setPower(-gamepad2.left_trigger*2/4);
                }
                if(gamepad1.right_trigger>0){
                    robot.portArm.setPower(gamepad1.right_trigger);
                    robot.starArm.setPower(gamepad2.right_trigger*2/4);
                }
                robot.portArm.setPower(-gamepad1.left_stick_y);
                robot.starArm.setPower(-gamepad2.left_stick_y*2/4);
            }
            if(extendyBoiTimerExtend +0.3<=getRuntime()&& extendyBoiTimerExtend +0.5>=getRuntime()){
                robot.extendyBoi.setPosition(extendyBoiExtend);
            }
            if(gamepad1.y){
                robot.elbow.setPosition(elbowNorminal);
                robot.extendyBoi.setPosition(extendyBoiRetract);
                robot.shoulderPort.setPosition(shoulderPortLift);
                robot.shoulderStar.setPosition(v3Hardware.shoulderStarLift);
                extendyBoiTimerExtend = getRuntime()+1.5;
                robot.wrist.setPosition(wristDown);
                wristLiftTimer = getRuntime();
                wristTimerDown = -3;
                shoulderTimerDown = -3;
                armDownTimer = -3;}
            if(wristLiftTimer + 1< getRuntime()&& wristLiftTimer+1.3>getRuntime()){
                robot.wrist.setPosition(wristLift);}
            if(robot.handTS.isPressed()|| gamepad1.a) {
                robot.door.setPosition(doorOpen);
                doorTimer = getRuntime();
                doorPos = "open";
            }else{
                robot.door.setPosition(doorClosed);
                doorPos = "closed";
            }
            if(doorTimer+1<getRuntime()&& doorTimer+1.2>getRuntime()){
                robot.shoulderStar.setPosition(shoulderStarScore - 0.04);
                robot.shoulderPort.setPosition(shoulderPortScore + 0.04);
                robot.elbow.setPosition(elbowNorminal);
                robot.starArm.setPower(-0.8);
                robot.portArm.setPower(-0.8);
                robot.extendyBoi.setPosition(extendyBoiTimeRetract);
                wristTimerDown = getRuntime();
                shoulderTimerDown = getRuntime();
                armDownTimer = getRuntime();
                armPos = "down";
            }
            if(gamepad1.left_bumper){
                robot.aimer.setPower(1);}
            else if(gamepad1.right_bumper){
                robot.aimer.setPower(-1);}
            else{
                robot.aimer.setPower(0);}
            if(gamepad1.x){
                robot.trigger.setPosition(1);}
            if(gamepad1.b){
                robot.trigger.setPosition(0);}
        }
    }
}