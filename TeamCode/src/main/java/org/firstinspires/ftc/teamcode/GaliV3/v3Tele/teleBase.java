package org.firstinspires.ftc.teamcode.GaliV3.v3Tele;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.GaliV3.v3Hardware;
@Config
@TeleOp
public class teleBase extends LinearOpMode {
    public v3Hardware robot = new v3Hardware();
    public static double shoulderPos = 0.5;
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        //robot.extendyBoi.setPosition(v3Hardware.extendyBoiRetract);
        //robot.elbow.setPosition(v3Hardware.elbowNorminal);
        //robot.shoulderStar.setPosition(v3Hardware.shoulderStarDown);
        //robot.shoulderPort.setPosition(v3Hardware.shoulderPortDown);
        //robot.wrist.setPosition(v3Hardware.wristDown);
        waitForStart();
        while (opModeIsActive()){
            robot.trigger.setPosition(shoulderPos);
        }
    }
}
