package org.firstinspires.ftc.teamcode.Tele;

import static org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem.ArmPos.*;
import static org.firstinspires.ftc.teamcode.GaliHardware.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.GaliHardware;
import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;

@TeleOp
public class GaliTele extends LinearOpMode {
    GaliHardware robot = new GaliHardware();
    ArmSubsystem.ArmPos armTarget = DOWN_FRONT;

    public double fingerPosPort = fingerPortClosed, fingerPosStar = fingerStarClosed,
            wristPosPort, wristPosStar, aimerPos = aimerDown, triggerPos = triggerUp, intakePower = 0;

    public boolean intakeOn = false;

    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();
    public void runOpMode() {
        robot.init(hardwareMap);
        waitForStart();

        while (opModeIsActive()) {
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            robot.fpd.setPower(-gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x);
            robot.bpd.setPower(-gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x);
            robot.fsd.setPower(-gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x);
            robot.bsd.setPower(-gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x);

            setArmPower(-gamepad2.left_stick_y);

            if(!previousGamepad1.back && gamepad1.back){ intakeOn = !intakeOn;}

            robot.intake.setPower(getHandPower());

            robot.wrist.setPosition(getWristPosPort());

            robot.fingerPort.setPosition(getFingerPosPort());
            robot.fingerStar.setPosition(getFingerPosStar());

            robot.aimer.setPosition(getAimerPos());
            robot.trigger.setPosition(getTriggerPos());

            telemetry.addData("BOW", robot.bpd.getCurrentPosition());
            telemetry.addData("POW", robot.fpd.getCurrentPosition());
            telemetry.addData("SOW", robot.SOW.getCurrentPosition());
            telemetry.update();
        }
    }
    public double getFingerPosPort(){
        if (gamepad2.left_bumper){
            fingerPosPort = fingerPortClosed;
        }
        if (gamepad2.left_trigger>0){
            fingerPosPort = fingerPortOpen;
        }
        return fingerPosPort;
    }

    public double getFingerPosStar(){
        if (gamepad2.right_bumper){
            fingerPosStar = fingerStarClosed;
        }
        if (gamepad2.right_trigger>0){
            fingerPosStar = fingerStarOpen;
        }
        return fingerPosStar;
    }

    public double getWristPosPort(){
        if (gamepad2.a){
            wristPosPort = wristUpPort;
        }
        if (gamepad2.b){
            wristPosPort = wristPickupPort;
        }
        if (gamepad2.dpad_left){
            wristPosPort = wristScoreLowPort;
        }
        if (gamepad2.dpad_up){
            wristPosPort = wristScoreHighPort;
        }
        return wristPosPort;
    }

    public double getWristPosStar(){
        if (gamepad2.a){
            wristPosStar = wristUpStar;
        }
        if (gamepad2.b){
            wristPosStar = wristPickupStar;
        }
        if (gamepad2.dpad_left){
            wristPosStar = wristScoreLowStar;
        }
        if(gamepad2.dpad_up){
            wristPosStar = wristScoreHighStar;
        }
        return wristPosStar;
    }

    public double getAimerPos(){
        if (gamepad1.left_bumper){
            aimerPos = aimerUp;
        }
        if (gamepad1.left_trigger>0){
            aimerPos = aimerDown;
        }
        return aimerPos;
    }

    public double getTriggerPos(){
        if (gamepad1.right_bumper){
            triggerPos = triggerDown;
        }
        if (gamepad1.right_trigger>0){
            triggerPos = triggerUp;
        }
        return triggerPos;
    }

    public ArmSubsystem.ArmPos getArmTarget(){
        if(gamepad2.dpad_up){
            armTarget = HIGH_FRONT;
        }
        if(gamepad2.dpad_left){
            armTarget = LOW_FRONT;
        }
        if(gamepad2.dpad_down){
            armTarget = DOWN_FRONT;
        }
        return armTarget;
    }

    public double getHandPower(){
        if(intakeOn){
            intakePower = .75;
        } else {
            intakePower = 0;
        }
        return intakePower;
    }

    /*public void elbowToPosition(ArmSubsystem.ArmPos targetPos){
        robot.elbow.setTargetPosition(targetPos.getElbowPos());
        robot.elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.elbow.setPower(1);
    }

    public void shoulderToPosition(ArmSubsystem.ArmPos targetPos){
        robot.shoulder.setTargetPosition(targetPos.getShoulderPos());
        robot.shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.shoulder.setPower(1);
    }
    public void armToPosition(ArmSubsystem.ArmPos targetPos){
        setArmTarget(targetPos.getElbowPos(), targetPos.getShoulderPos());
        setArmMode(DcMotor.RunMode.RUN_TO_POSITION);
        setArmPower(.25);
    }

    public void setArmMode(DcMotor.RunMode runMode){
        robot.elbow.setMode(runMode);
        robot.shoulder.setMode(runMode);
    }

    public void setArmTarget(int targetElbow, int targetShoulder){
        robot.elbow.setTargetPosition(targetElbow);
        robot.shoulder.setTargetPosition(targetShoulder);
    }

    public void resetArm(){
        robot.elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.elbow.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.shoulder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }*/

    public void setArmPower(double power) {
        robot.armPort.setPower(power);
        robot.armStar.setPower(power);
    }
}