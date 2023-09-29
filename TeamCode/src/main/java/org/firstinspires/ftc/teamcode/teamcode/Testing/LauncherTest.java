package org.firstinspires.ftc.teamcode.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.teamcode.GaliHardware;

@TeleOp
public class LauncherTest extends LinearOpMode {
    GaliHardware robot = new GaliHardware();
    public void runOpMode() {
        robot.init(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            if(gamepad1.b){
                robot.launcherStopper.setPosition(robot.launcherRelease);
            }
            if(gamepad1.x){
                robot.launcherStopper.setPosition(robot.launcherHold);
            }
            if (gamepad1.y){
                robot.launcherExtender.setPosition(robot.launcherExtenderUp);
            }
            if (gamepad1.a){
                robot.launcherExtender.setPosition(robot.launcherExtenderDown);
            }
        }

    }
}
