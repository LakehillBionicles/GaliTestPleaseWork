package org.firstinspires.ftc.teamcode.GaliV3.v3Tele;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.GaliV3.v3Hardware;

@TeleOp
@Config
public class manualModeTele extends teleBase{
    public static double extendyBoiPos = 0.5;
    public static double wristPos = 0.5;
    public static double shoulderPortPos = 0.5;
    public static double shoulderStarPos = 0.5;
    public static double doorPos = 0.5;
    public static double elbowPos = 0.5;
    public static double intakePower = 0;
    public static double drivePower = 1;

    public static double triggerPos = 0;


    @Override
    public void runOpMode(){
        robot.init(hardwareMap);
        robot.door.setPosition(doorPos);
        robot.extendyBoi.setPosition(extendyBoiPos);
        robot.elbow.setPosition(elbowPos);
        robot.shoulderStar.setPosition(shoulderStarPos);
        robot.shoulderPort.setPosition(shoulderPortPos);
        robot.wrist.setPosition(wristPos);
        waitForStart();
        while (opModeIsActive()) {
            robot.extendyBoi.setPosition(extendyBoiPos);
            robot.wrist.setPosition(wristPos);
            robot.shoulderPort.setPosition(shoulderPortPos);
            robot.shoulderStar.setPosition(shoulderStarPos);
            robot.door.setPosition(doorPos);
            robot.elbow.setPosition(elbowPos);
            /*
            robot.fpd.setPower((-gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x) * drivePower);
            robot.bpd.setPower((-gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x) * drivePower);
            robot.fsd.setPower((-gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x) * drivePower);
            robot.bsd.setPower((-gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x) * drivePower);
            if(gamepad1.x){
                robot.intake.setPower(intakePower);
            }
            if(gamepad1.b){
                robot.intake.setPower(0);
            }
            if(gamepad1.y){
                robot.intake.setPower(-intakePower);
            }
            if(gamepad1.dpad_up){
                robot.aimer.setPower(0.3);
            }
            if(gamepad1.dpad_down){
                robot.aimer.setPower(-0.3);
            }
            if(gamepad1.right_bumper){
                robot.trigger.setPosition(triggerPos);
            }
            robot.portArm.setPower(-gamepad2.left_stick_y);
            robot.starArm.setPower(-gamepad2.left_stick_y);

             */
        }
    }
}
