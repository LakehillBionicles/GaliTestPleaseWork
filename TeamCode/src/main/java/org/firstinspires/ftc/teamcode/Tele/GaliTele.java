package org.firstinspires.ftc.teamcode.Tele;

import static org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem.ArmPos.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.GaliHardware;
import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;

@TeleOp
public class GaliTele extends LinearOpMode {
    GaliHardware robot = new GaliHardware();
    ArmSubsystem.ArmPos armTarget = DOWN_FRONT;

    public void runOpMode() {
        robot.init(hardwareMap);

        resetArm();
        robot.POW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.BOW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bsd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        waitForStart();

        while (opModeIsActive()) {
            robot.launcherStopper.setPosition(robot.launcherHold);
            robot.launcherExtender.setPosition(robot.launcherExtenderDown);
            robot.fpd.setPower(-gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x);
            robot.bpd.setPower(-gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x);
            robot.fsd.setPower(-gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x);
            robot.bsd.setPower(-gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x);

            armToPosition(getArmTarget());

            /*if(gamepad2.left_bumper){
                robot.handStar.setTargetPosition(robot.fingerStarClosed);
            }
            if (gamepad2.right_bumper){
                robot.fingerStar.setPosition(robot.fingerStarOpen);
            }

             */
            if (gamepad2.left_trigger > 0){
                robot.handPort.setPower(0.75);
            }
            if (gamepad2.right_trigger > 0){
                robot.handStar.setPower(0.75);
            }
            if (gamepad2.a){
                robot.wrist.setPosition(robot.wristPickup);
            }
            if (gamepad2.b){
                robot.wrist.setPosition(robot.wristScore);
            }

            telemetry.addData("POW", robot.POW.getCurrentPosition());
            telemetry.addData("BOW", robot.BOW.getCurrentPosition());
            telemetry.addData("SOW", robot.bsd.getCurrentPosition());
            telemetry.update();
        }
    }

    public ArmSubsystem.ArmPos getArmTarget(){
        if(gamepad2.dpad_up){
            armTarget = HIGH_FRONT;
        }
        if(gamepad2.dpad_right){
            armTarget = MID_FRONT;
        }
        if(gamepad2.dpad_left){
            armTarget = LOW_FRONT;
        }
        if(gamepad2.dpad_down){
            armTarget = DOWN_FRONT;
        }
        return armTarget;
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