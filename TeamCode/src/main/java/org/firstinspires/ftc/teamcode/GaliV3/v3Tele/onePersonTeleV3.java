package org.firstinspires.ftc.teamcode.GaliV3.v3Tele;

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
import static org.firstinspires.ftc.teamcode.GaliV3.v3Hardware.triggerRelease;
import static org.firstinspires.ftc.teamcode.GaliV3.v3Hardware.wristDown;
import static org.firstinspires.ftc.teamcode.GaliV3.v3Hardware.wristPort;
import static org.firstinspires.ftc.teamcode.GaliV3.v3Hardware.wristStar;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.GaliV3.v3Hardware;

public class onePersonTeleV3 extends teleBase {
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
    double wristLiftTimer = -3;
    String armPos = "down";
    double aimerTimer = -3;
    double runtime = 0;
    boolean flipperDown = true;


    @Override
    public void runOpMode() {
        runtime = getRuntime();
        robot.init(hardwareMap);
        robot.door.setPosition(v3Hardware.doorClosed);
        robot.extendyBoi.setPosition(v3Hardware.extendyBoiDown);
        robot.elbow.setPosition(v3Hardware.elbowNorminal);
        //robot.shoulderStar.setPosition(v3Hardware.shoulderStarDown);
        robot.shoulderPort.setPosition(v3Hardware.shoulderPortDown);
        robot.wrist.setPosition(v3Hardware.wristDown);
        robot.flipper.setPosition(v3Hardware.flipDown);
        waitForStart();
        while (opModeIsActive()) {
            runtime = getRuntime();
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);
            intakeSpit = gamepad1.x;
            if (!previousGamepad1.back && gamepad1.back) {
                intakeOn = !intakeOn;
            }
            if (!previousGamepad1.b && gamepad1.b) {
                flipperDown = !flipperDown;
            }
            robot.fpd.setPower((-gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x) * drivePower);
            robot.bpd.setPower((-gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x) * drivePower);
            robot.fsd.setPower((-gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x) * drivePower);
            robot.bsd.setPower((-gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x) * drivePower);
            if (intakeSpit) {
                robot.intake.setPower(-org.firstinspires.ftc.teamcode.GaliV3.v3Hardware.intakeSpeed);
            } else if (intakeOn&&flipperDown) {
                robot.intake.setPower(org.firstinspires.ftc.teamcode.GaliV3.v3Hardware.intakeSpeed);
            } else {
                robot.intake.setPower(0);
            }
            if (gamepad1.dpad_up) {
                robot.wrist.setPosition(wristDown);
                robot.elbow.setPosition(elbowNorminal);
                //robot.shoulderStar.setPosition(shoulderStarScore);
                robot.shoulderPort.setPosition(shoulderPortScore);
                robot.extendyBoi.setPosition(extendyBoiRetract);
                wristTimerDown = -3;
                shoulderTimerDown = -3;
                armDownTimer = -3;
                armPos = "up";
            }
            if (gamepad1.dpad_left) {
                if (robot.shoulderPort.getPosition() == shoulderStarDown || armPos.equals("down")) {
                    //robot.shoulderStar.setPosition(shoulderStarScore);
                    robot.shoulderPort.setPosition(shoulderPortScore);
                    robot.extendyBoi.setPosition(extendyBoiRetract);
                    wristTimerDown = -3;
                    shoulderTimerDown = -3;
                    armDownTimer = -3;
                    elbowPortTimer = runtime;

                } else if (elbowPortTimer + 2.2 < runtime) {
                    robot.wrist.setPosition(wristPort);
                    robot.elbow.setPosition(elbowPort);
                    //robot.shoulderStar.setPosition(shoulderStarScore);
                    robot.shoulderPort.setPosition(shoulderPortScore);
                    extendyBoiTimerExtend = runtime;
                    armUpTimer = runtime;
                }
                armPos = "port";
            }
            if (gamepad1.dpad_right) {
                if (robot.shoulderPort.getPosition() == shoulderStarDown || armPos.equals("down")) {
                    //robot.shoulderStar.setPosition(shoulderStarScore);
                    robot.shoulderPort.setPosition(shoulderPortScore);
                    robot.extendyBoi.setPosition(extendyBoiRetract);
                    wristTimerDown = -3;
                    shoulderTimerDown = -3;
                    armDownTimer = -3;
                    elbowStarTimer = runtime;
                } else if (elbowStarTimer + 2.2 < runtime) {
                    robot.wrist.setPosition(wristStar);
                    robot.elbow.setPosition(elbowStar);
                    //robot.shoulderStar.setPosition(shoulderStarScore);
                    robot.shoulderPort.setPosition(shoulderPortScore);
                    extendyBoiTimerExtend = runtime;
                    armUpTimer = runtime;
                }
                armPos = "star";
            }
            if ((elbowStarTimer + 0.5 < runtime && elbowStarTimer + 0.7 > runtime) || (elbowPortTimer + 0.5 < runtime && elbowPortTimer + 0.7 > runtime)) {
                robot.portArm.setPower(0.5);
                robot.starArm.setPower(0.4);
            }
            if (elbowStarTimer + 2.2 < runtime && elbowStarTimer + 2.3 > runtime) {
                robot.wrist.setPosition(wristStar);
                robot.elbow.setPosition(elbowStar);
                extendyBoiTimerExtend = runtime;
                robot.portArm.setPower(0);
                robot.starArm.setPower(0);
            }
            if (elbowPortTimer + 2.2 < runtime && elbowPortTimer + 2.3 > runtime) {
                robot.wrist.setPosition(wristPort);
                robot.portArm.setPower(0);
                robot.portArm.setPower(0);
                robot.elbow.setPosition(elbowPort);
                extendyBoiTimerExtend = runtime;
            }
            if (gamepad1.dpad_down) {
                if (armPos.equals("star") || armPos.equals("port") || robot.elbow.getPosition() == elbowStar || robot.elbow.getPosition() == elbowPort || robot.wrist.getPosition() == wristPort || robot.wrist.getPosition() == wristStar) {
                    //robot.shoulderStar.setPosition(shoulderStarScore - 0.04);
                    robot.shoulderPort.setPosition(shoulderPortScore + 0.04);
                    robot.elbow.setPosition(elbowNorminal);
                    robot.starArm.setPower(-0.8);
                    robot.portArm.setPower(-0.8);
                    robot.extendyBoi.setPosition(extendyBoiTimeRetract);
                    wristTimerDown = runtime;
                    shoulderTimerDown = runtime;
                    armDownTimer = runtime;
                } else if (wristTimerDown + 1 < runtime) {
                    robot.wrist.setPosition(wristDown);
                    robot.elbow.setPosition(elbowNorminal);
                    //robot.shoulderStar.setPosition(shoulderStarDown);
                    robot.shoulderPort.setPosition(shoulderPortDown);
                    robot.extendyBoi.setPosition(extendyBoiDown);
                }
                armPos = "down";
            }
            if (wristTimerDown + 0.5 < runtime && wristTimerDown + 1.2 > runtime) {
                robot.wrist.setPosition(wristDown);
            }
            if (wristTimerDown + 1 < runtime && wristTimerDown + 1.2 > runtime) {
                //robot.shoulderStar.setPosition(shoulderStarDown);
                robot.shoulderPort.setPosition(shoulderPortDown);
                robot.extendyBoi.setPosition(extendyBoiDown);
            }
            if (armDownTimer + 1.5 < runtime) {
                robot.portArm.setPower(gamepad1.left_trigger - gamepad1.right_trigger);
                robot.starArm.setPower(gamepad1.left_trigger - gamepad1.right_trigger);
            }
            if (extendyBoiTimerExtend + 0.3 <= runtime && extendyBoiTimerExtend + 0.5 >= runtime) {
                robot.extendyBoi.setPosition(extendyBoiExtend);
            }
            if (gamepad1.right_bumper) {
                robot.elbow.setPosition(elbowNorminal);
                robot.extendyBoi.setPosition(extendyBoiRetract);
                robot.shoulderPort.setPosition(shoulderPortLift);
                //robot.shoulderStar.setPosition(v3Hardware.shoulderStarLift);
                extendyBoiTimerExtend = runtime + 1.5;
                robot.wrist.setPosition(wristDown);
                wristLiftTimer = runtime;
                wristTimerDown = -3;
                shoulderTimerDown = -3;
                armDownTimer = -3;
            }
            if (wristLiftTimer + 1 < runtime && wristLiftTimer + 1.3 > runtime) {
                robot.wrist.setPosition(wristPort);
            }
            if (robot.handTS.isPressed() || gamepad1.a) {
                robot.door.setPosition(doorOpen);
                doorTimer = runtime;
                doorPos = "open";
            } else {
                robot.door.setPosition(doorClosed);
                doorPos = "closed";
            }
            if (doorTimer + 0.8 < runtime && doorTimer + 1 > runtime) {
                if (armPos.equals("star") || armPos.equals("port") || robot.elbow.getPosition() == elbowStar || robot.elbow.getPosition() == elbowPort || robot.wrist.getPosition() == wristPort || robot.wrist.getPosition() == wristStar) {
                    robot.shoulderPort.setPosition(shoulderPortScore + 0.04);
                    robot.elbow.setPosition(elbowNorminal);
                    robot.starArm.setPower(-0.8);
                    robot.portArm.setPower(-0.8);
                    robot.extendyBoi.setPosition(extendyBoiTimeRetract);
                    wristTimerDown = runtime;
                    shoulderTimerDown = runtime;
                    armDownTimer = runtime;
                } else if (wristTimerDown + 1 < runtime) {
                    robot.wrist.setPosition(wristDown);
                    robot.elbow.setPosition(elbowNorminal);
                    //robot.shoulderStar.setPosition(shoulderStarDown);
                    robot.shoulderPort.setPosition(shoulderPortDown);
                    robot.extendyBoi.setPosition(extendyBoiDown);
                }
                armPos = "down";
            }
            if (gamepad1.left_bumper) {
                robot.aimer.setPower(1);
            }
            if (robot.aimer.getPower() > 1 && !gamepad1.left_bumper) {
                robot.trigger.setPosition(triggerRelease);
                aimerTimer = runtime;
            }
            if (aimerTimer + 0.5 < runtime && aimerTimer + 0.8 > runtime) {
                robot.aimer.setPower(-1);
            }
            if (aimerTimer + 2 < runtime && aimerTimer + 2.3 > runtime) {
                robot.aimer.setPower(0);
            }
            if (flipperDown) {
                robot.flipper.setPosition(v3Hardware.flipDown);}
            else{
                robot.flipper.setPosition(v3Hardware.flipUp);}
        }
    }
}