package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
@Disabled
public class workgoddamit extends LinearOpMode {
    public void runOpMode() {
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("work!!", "fuck yeah");
            telemetry.update();
        }
    }
}
