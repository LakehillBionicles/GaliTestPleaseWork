package org.firstinspires.ftc.teamcode.GaliV3.v3Tele;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.GaliV3.v3Hardware;
@TeleOp
@Config
public class intakeGoBurrr extends teleBase {

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();
        while(opModeIsActive()) {

            if (gamepad1.x) {
                robot.intake.setPower(v3Hardware.intakeSpeed);
            }
            else if (gamepad1.b) {
                robot.intake.setPower(-v3Hardware.intakeSpeed);
            }
            else{
                robot.intake.setPower(0);
            }
        }
    }
}
