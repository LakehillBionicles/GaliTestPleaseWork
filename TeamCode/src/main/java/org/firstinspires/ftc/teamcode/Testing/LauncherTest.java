package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.GaliHardware;
@Config
@TeleOp
public class LauncherTest extends LinearOpMode {
    GaliHardware robot = new GaliHardware();
    private double counter = 0;
    public static double highAngle = 0.67;
    public void runOpMode() {
        robot.init(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();


        while (opModeIsActive()) {
            if(gamepad1.b){
                //robot.launcherStopper.setPosition(robot.launcherRelease);
            }
            if(gamepad1.x){
                //robot.launcherStopper.setPosition(robot.launcherHold);
            }
            if (gamepad1.y){
                //robot.launcherExtender.setPosition(highAngle-counter);
            }
            if (gamepad1.a){
                //robot.launcherExtender.setPosition(robot.launcherExtenderDown);
            }

            if (gamepad1.dpad_up&&(getRuntime()>0.6)){
                resetRuntime();
                counter = counter + 0.01;
            }
            if(gamepad1.dpad_down&&getRuntime()>0.6){
                resetRuntime();
                counter = counter -0.01;
            }
            //telemetry.addData("heightOfLauncher", robot.launcherExtenderUp + counter);
            telemetry.addData("counter", counter);
            telemetry.update();
        }

    }
}
