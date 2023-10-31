package org.firstinspires.ftc.teamcode.Tele;

import static org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem.ArmPos.*;
import static org.firstinspires.ftc.teamcode.GaliHardware.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.GaliHardware;
import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;

import java.util.Objects;
@Config

@TeleOp
public class GaliTele extends LinearOpMode {
    GaliHardware robot = new GaliHardware();
    ArmSubsystem.ArmPos armTarget = DOWN_FRONT;
    public static int wristUp = 0;
    public static int wristPickup = -1880;
    public static double fingerPortOpen = 0.6, fingerPortClosed = .15;
    public static double fingerStarOpen = 0.6, fingerStarClosed = .15;

    public static int wristScore = -880;

    public double fingerPosPort = fingerPortClosed, fingerPosStar = fingerStarClosed,
            wristPosPort, wristPosStar, aimerPos = aimerDown, triggerPos = triggerUp, handPower = 0;

    public boolean handOn = false;
    public String shouldHalf = "half";
    public String elbowHalf = "half";

    public String manual = "no";
    public String elbowAtPosition = "no";

    public String shouldAtPosition = "no";

    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();
    public void runOpMode() {
        robot.init(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        resetArm();

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

            //elbowToPosition(getArmTarget());
            if(Objects.equals(manual, "no")) {
            if (Objects.equals(shouldHalf, "half")) {
                shoulderToHalfPosition(getArmTarget());
            }
            if (Objects.equals(shouldHalf, "there")) {
                shoulderToPosition(getArmTarget());
            }
            //Objects.equals(shouldHalf, "there")&&
            //&& Objects.equals(shouldHalf, "there")
                if (Objects.equals(elbowHalf, "half")) {
                    elbowToHalfPosition(getArmTarget());
                }
                if (Objects.equals(elbowHalf, "there")) {
                    elbowToPosition(getArmTarget());
                }
            }


            telemetry.addData("value:", String.valueOf(robot.wrist.getCurrentPosition()));
            telemetry.update();
            wristToPosition();
            robot.handStar.setPower(gamepad1.left_trigger - gamepad1.right_trigger);
            robot.handPort.setPower(gamepad1.left_trigger - gamepad1.right_trigger);

            if (!previousGamepad1.back && gamepad1.back) {
                handOn = !handOn;
            }

            robot.handPort.setDirection(CRServo.Direction.FORWARD);
            //robot.handStar.setPower(getHandPower());

            robot.fingerPort.setPosition(getFingerPosPort());
            robot.fingerStar.setPosition(getFingerPosStar());
            if (gamepad1.left_bumper) {
                robot.aimer.setPosition(1);
            }
            if (gamepad1.left_trigger > 0) {
                robot.aimer.setPosition(0);
            }
            //robot.aimer.setPosition(getAimerPos());
            robot.trigger.setPosition(getTriggerPos());
            if(gamepad1.y){
                manual = "manual";
            }
            if(gamepad1.a){
                manual = "no";
            }
            if(Objects.equals(manual, "manual")){
                robot.shoulder.setPower(gamepad2.left_stick_y);
                robot.elbow.setPower(gamepad2.right_stick_y);}
            telemetry.addData("manual:", manual);
            telemetry.update();
            /*
            if(gamepad2.left_stick_y>0.2||gamepad2.left_stick_y<0.2){
                robot.shoulder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.shoulder.setPower(gamepad2.left_stick_y);
                shouldAtPosition = "no";}
            if(gamepad2.left_stick_y<=0.2&&gamepad2.left_stick_y>=-0.2&& Objects.equals(shouldAtPosition, "no")){
                robot.shoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.shoulder.setTargetPosition(robot.shoulder.getCurrentPosition());
                robot.shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                shouldAtPosition = "yes";}
            if(gamepad2.right_stick_y>0.2||gamepad2.right_stick_y<0.2){
                robot.elbow.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.elbow.setPower(gamepad2.right_stick_y);
                elbowAtPosition = ("no");}
            if(gamepad2.right_stick_y<=0.2&&gamepad2.right_stick_y>=-0.2&& Objects.equals(elbowAtPosition, "no")){
                robot.elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.elbow.setTargetPosition(robot.elbow.getCurrentPosition());
                robot.elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elbowAtPosition = "yes";}

             */
            /*
            if(gamepad2.y){
                robot.wrist.setTargetPosition(wristUp);
                robot.wrist.setMode(DcMotor.RunMode.RUN_TO_POSITION);


                robot.wrist.setPower(1);
            }if(gamepad2.a){

                robot.wrist.setTargetPosition(wristPickup);
                robot.wrist.setMode(DcMotor.RunMode.RUN_TO_POSITION);


                robot.wrist.setPower(-1);
            }

             */

        }
    }

    public void wristToPosition(){
        if(gamepad2.y){
            robot.wrist.setTargetPosition(wristUp);
            robot.wrist.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.wrist.setPower(1);
        }if(gamepad2.a){
            robot.wrist.setTargetPosition(wristPickup);
            robot.wrist.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.wrist.setPower(1);
        }if(gamepad2.b){
            robot.wrist.setTargetPosition(wristScore);
            robot.wrist.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.wrist.setPower(1);
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
        if (gamepad2.left_bumper){
            aimerPos = aimerUp;
        }
        if (gamepad2.left_trigger>0){
            aimerPos = aimerDown;
        }
        return aimerPos;
    }

    public double getTriggerPos(){
        if (gamepad1.dpad_up){
            triggerPos = triggerDown;
        }
        if (gamepad1.dpad_down){
            triggerPos = triggerUp;
        }
        return triggerPos;
    }

    public ArmSubsystem.ArmPos getArmTarget(){
        if(gamepad2.dpad_up){
            armTarget = HIGH_FRONT;
            shouldHalf = "half";
            elbowHalf = "half";
        }
        if(gamepad2.dpad_left){
            armTarget = LOW_FRONT;
            shouldHalf = "half";
            elbowHalf = "half";
        }
        if(gamepad2.dpad_down){
            armTarget = DOWN_FRONT;
            shouldHalf = "half";
            elbowHalf = "half";
        }
        return armTarget;
    }

    public double getHandPower(){
        if(handOn){
            handPower = .75;
        } else {
            handPower = 0;
        }
        return handPower;
    }

    public void elbowToPosition(ArmSubsystem.ArmPos targetPos){
        robot.elbow.setTargetPosition(targetPos.getElbowPos());
        robot.elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.elbow.setPower(1);
    }
    public void elbowToHalfPosition(ArmSubsystem.ArmPos targetPos){
        robot.elbow.setTargetPosition(targetPos.getElbowPos() / 2);
        robot.elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.elbow.setPower(1);
        if(robot.elbow.getCurrentPosition() * 0.95>= (targetPos.getElbowPos() / 2)&& robot.elbow.getCurrentPosition()<= (targetPos.getElbowPos() / 2)*1.05) {
            elbowHalf = "there";
        }
    }

    public void shoulderToHalfPosition(ArmSubsystem.ArmPos targetPos){
        robot.shoulder.setTargetPosition(targetPos.getShoulderPos() / 2);
        robot.shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.shoulder.setPower(1);
        if(robot.shoulder.getCurrentPosition() * 0.95>= (targetPos.getShoulderPos() / 2)&& robot.shoulder.getCurrentPosition()<= (targetPos.getShoulderPos() / 2)*1.05) {
            shouldHalf = "there";
        }
        sleep(200);
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
    }

    public void setArmPower(double power) {
        robot.elbow.setPower(power);
        robot.shoulder.setPower(power);
    }
}