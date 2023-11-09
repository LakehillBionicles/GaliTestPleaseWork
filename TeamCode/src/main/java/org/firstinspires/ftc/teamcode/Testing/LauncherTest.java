package org.firstinspires.ftc.teamcode.Testing;

import static org.firstinspires.ftc.teamcode.GaliHardware.aimerDown;
import static org.firstinspires.ftc.teamcode.GaliHardware.triggerDown;
import static org.firstinspires.ftc.teamcode.GaliHardware.triggerUp;

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
                //robot.aimer.setPosition(highAngle);
            }
            if(gamepad1.x){
                //robot.aimer.setPosition(aimerDown);
            }
            if (gamepad1.y){
                robot.trigger.setPosition(triggerUp);
            }
            if (gamepad1.a){
                robot.trigger.setPosition(triggerDown);
            }

            if (gamepad1.dpad_up&&(getRuntime()>0.6)){
                resetRuntime();
                counter = counter + 0.01;
            }
            if(gamepad1.dpad_down&&getRuntime()>0.6){
                resetRuntime();
                counter = counter -0.01;
            }
        }

    }
}
