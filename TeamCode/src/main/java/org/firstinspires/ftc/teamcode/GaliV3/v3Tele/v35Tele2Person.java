package org.firstinspires.ftc.teamcode.GaliV3.v3Tele;

import static org.firstinspires.ftc.teamcode.GaliV3.v3Hardware.elbowNorminal;
import static org.firstinspires.ftc.teamcode.GaliV3.v3Hardware.extendyBoiRetract;
import static org.firstinspires.ftc.teamcode.GaliV3.v3Hardware.shoulderPortLift;
import static org.firstinspires.ftc.teamcode.GaliV3.v3Hardware.wristDown;
import static org.firstinspires.ftc.teamcode.GaliV3.v3Hardware.wristLift;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.GaliV3.v3Hardware;
@TeleOp
public class v35Tele2Person extends teleBase {
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
    double launcherTimerDown = -3;
    double starTimer = -3;
    double portTimer = -3;
    String armPos = "down";
    double intakeTimer = -3;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        robot.door.setPosition(v3Hardware.doorClosed);
        robot.extendyBoi.setPosition(v3Hardware.extendyBoiDown);
        robot.elbow.setPosition(v3Hardware.elbowNorminal);
        robot.shoulderStar.setPosition(v3Hardware.shoulderStarDown);
        robot.shoulderPort.setPosition(v3Hardware.shoulderPortDown);
        robot.wrist.setPosition(v3Hardware.wristDown);
        robot.flipper.setPosition(v3Hardware.flipDown);
        waitForStart();
        while (opModeIsActive()) {
            //Non conditionals
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);
            robot.fpd.setPower((-gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x) * drivePower);
            robot.bpd.setPower((-gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x) * drivePower);
            robot.fsd.setPower((-gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x) * drivePower);
            robot.bsd.setPower((-gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x) * drivePower);
            //Intake stuff that works no touch
            intakeSpit = gamepad1.x;
                    //|| (intakeOn && robot.intake.getVelocity(AngleUnit.RADIANS) >= -0.3 && intakeTimer + 2 < getRuntime());
            if (!previousGamepad1.back && gamepad1.back) {
                intakeOn = !intakeOn;
                /*if(intakeOn){
                    intakeTimer = getRuntime();}

                 */
            }
            if (intakeSpit) {
                robot.intake.setPower(-org.firstinspires.ftc.teamcode.GaliV3.v3Hardware.intakeSpeed);
            } else if (intakeOn) {
                robot.intake.setPower(org.firstinspires.ftc.teamcode.GaliV3.v3Hardware.intakeSpeed);
            } else {
                robot.intake.setPower(0);
            }
            //Stupid launcher stuff no touch only austin touch. Austin Smart.
            if (gamepad1.dpad_up) {
                robot.aimer.setPower(v3Hardware.aimerUp);
                launcherTimerDown = -3;
            }
            if (gamepad1.dpad_down) {
                robot.aimer.setPower(-v3Hardware.aimerUp);
            }
            if (gamepad1.dpad_right) {
                robot.aimer.setPower(0);
            }
            //Might want to flip
            if (gamepad1.left_trigger > 0) {
                robot.trigger.setPosition(v3Hardware.triggerHold);
            }
            //Timer is to make us launch and then bring down launcher. Austin sometimes dumb and forgets
            if (gamepad1.right_bumper && gamepad1.left_bumper) {
                robot.trigger.setPosition(v3Hardware.triggerRelease);
                launcherTimerDown = getRuntime();
            }
            if (launcherTimerDown + 1 < getRuntime() && launcherTimerDown + 1.3 > getRuntime()) {
                robot.aimer.setPower(-v3Hardware.aimerUp);
            } else if (launcherTimerDown + 3 < getRuntime()&&launcherTimerDown+3.5>getRuntime()) {
                robot.aimer.setPower(0);
            }
            //Door stuff
            if (gamepad2.a||robot.handTS.isPressed()) {
                robot.door.setPosition(v3Hardware.doorOpen);
            } else {
                robot.door.setPosition(v3Hardware.doorClosed);
            }
            //Fancy stuff ask musselan. She smart
            if (gamepad2.dpad_down) {
                robot.elbow.setPosition(v3Hardware.elbowNorminal);
                robot.extendyBoi.setPosition(v3Hardware.extendyBoiRetract);
                if (armPos.equals("up")||armPos.equals("down")) {
                    robot.shoulderPort.setPosition(v3Hardware.shoulderPortDown);
                    robot.shoulderStar.setPosition(v3Hardware.shoulderStarDown);
                    robot.wrist.setPosition(v3Hardware.wristDown);
                    armDownTimer = getRuntime();
                } else {
                    robot.shoulderPort.setPosition(v3Hardware.shoulderPortScore + 0.05);
                    robot.shoulderStar.setPosition(v3Hardware.shoulderStarScore - 0.05);
                    robot.wrist.setPosition(v3Hardware.wristDown);
                    armDownTimer = getRuntime() + 1.5;
                    wristTimerDown = getRuntime();
                }
                armPos = "down";
            } else if (gamepad2.dpad_up) {
                armPos = "up";
                robot.elbow.setPosition(v3Hardware.elbowNorminal);
                robot.extendyBoi.setPosition(v3Hardware.extendyBoiRetract);
                robot.shoulderPort.setPosition(v3Hardware.shoulderPortScore);
                robot.shoulderStar.setPosition(v3Hardware.shoulderStarScore);
                robot.wrist.setPosition(v3Hardware.wristDown);
            }
            else if (gamepad2.dpad_left) {
                robot.extendyBoi.setPosition(v3Hardware.extendyBoiRetract);
                if (armPos.equals("down")) {
                    robot.shoulderPort.setPosition(v3Hardware.shoulderPortScore);
                    robot.shoulderStar.setPosition(v3Hardware.shoulderStarScore);
                    robot.wrist.setPosition(v3Hardware.wristDown);
                    portTimer = getRuntime();
                    armUpTimer = getRuntime();
                } else {
                    robot.shoulderPort.setPosition(v3Hardware.shoulderPortScore);
                    robot.shoulderStar.setPosition(v3Hardware.shoulderStarScore);
                    robot.wrist.setPosition(v3Hardware.wristPort);
                    armUpTimer = getRuntime();
                    robot.elbow.setPosition(v3Hardware.elbowPort);
                    extendyBoiTimerExtend = getRuntime();
                }
                armPos = "left";
            } else if (gamepad2.dpad_right) {
                if (armPos.equals("down")) {
                    robot.shoulderPort.setPosition(v3Hardware.shoulderPortScore);
                    robot.shoulderStar.setPosition(v3Hardware.shoulderStarScore);
                    robot.wrist.setPosition(v3Hardware.wristDown);
                    starTimer = getRuntime();
                    armUpTimer = getRuntime();
                } else {
                    robot.shoulderPort.setPosition(v3Hardware.shoulderPortScore);
                    robot.shoulderStar.setPosition(v3Hardware.shoulderStarScore);
                    robot.wrist.setPosition(v3Hardware.wristStar);
                    armUpTimer = getRuntime();
                    extendyBoiTimerExtend = getRuntime();
                    robot.elbow.setPosition(v3Hardware.elbowStar);
                }
                armPos = "right";
            }
            else if (gamepad2.y) {
                robot.elbow.setPosition(elbowNorminal);
                robot.extendyBoi.setPosition(extendyBoiRetract);
                robot.shoulderPort.setPosition(shoulderPortLift);
                robot.shoulderStar.setPosition(v3Hardware.shoulderStarLift);
                robot.wrist.setPosition(wristDown);
                extendyBoiTimerExtend = getRuntime() + 1.5;
                wristLiftTimer = getRuntime();
                wristTimerDown = -3;
                shoulderTimerDown = -3;
                armDownTimer = -3;
                armPos = "lift";
            }
            telemetry.addData("armpos", armPos);
            telemetry.update();
            if (wristLiftTimer + 1 < getRuntime() && wristLiftTimer + 1.3 > getRuntime()) {
                robot.wrist.setPosition(wristLift);
            }
            //Really stupid timer stuff no touch besides austin and musselan.
            if (wristTimerDown + 0.5 < getRuntime() && wristTimerDown + 0.7 > getRuntime()) {
                robot.wrist.setPosition(v3Hardware.wristDown);
            }
            if (wristTimerDown + 1 < getRuntime() && wristTimerDown + 1.3 > getRuntime()) {
                robot.shoulderStar.setPosition(v3Hardware.shoulderStarDown);
                robot.shoulderPort.setPosition(v3Hardware.shoulderPortDown);
            }
            if ((starTimer + 1.6 < getRuntime() && starTimer + 1.9 > getRuntime()) || (portTimer + 1.6 < getRuntime() && portTimer + 1.9 > getRuntime())) {
                if (armPos.equals("star")) {
                    robot.elbow.setPosition(v3Hardware.elbowStar);
                    robot.wrist.setPosition(v3Hardware.wristStar);
                } else if(armPos.equals("port")){
                    robot.elbow.setPosition(v3Hardware.elbowPort);
                    robot.wrist.setPosition(v3Hardware.wristPort);
                }
                extendyBoiTimerExtend = getRuntime();
                armUpTimer = getRuntime();
            }
            if (armUpTimer + 1 < getRuntime() && armUpTimer + 1.5 > getRuntime()) {
                robot.portArm.setPower(1);
                robot.starArm.setPower(0.5);
            }else {
                robot.portArm.setPower(-gamepad2.left_stick_y);
                //Star arm seems more power don't know this is quick fix to stop belt skipping
                robot.starArm.setPower(-gamepad2.left_stick_y * ((double) 2 / 4));
            }
            if (extendyBoiTimerExtend + 0.7 < getRuntime() && extendyBoiTimerExtend + 0.9 > getRuntime()) {
                robot.extendyBoi.setPosition(v3Hardware.extendyBoiExtend);}
            if(gamepad1.y){
                robot.flipper.setPosition(v3Hardware.flipUp);}
            if(gamepad1.a){
                robot.flipper.setPosition(v3Hardware.flipDown);}
            //robot.starArm.setPower(0);
            //robot.starArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }
}