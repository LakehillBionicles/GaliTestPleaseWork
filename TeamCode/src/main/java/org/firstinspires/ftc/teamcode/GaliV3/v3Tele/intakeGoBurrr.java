package org.firstinspires.ftc.teamcode.GaliV3.v3Tele;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.GaliV3.v3Hardware;
@TeleOp
@Config
public class intakeGoBurrr extends teleBase {
    String distanceString = "not seen";
    double distanceTimer = -3;
    double counter = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        waitForStart();
        if(opModeIsActive()) {
            resetRuntime();
            robot.intake.setPower(v3Hardware.intakeSpeed);
            robot.intake.setPower(-v3Hardware.intakeSpeed);
            if(gamepad1.dpad_down){
                robot.shoulderPort.setPosition(v3Hardware.shoulderPortDown);
                robot.shoulderStar.setPosition(v3Hardware.shoulderStarDown);
            }
            if(gamepad1.dpad_up){
                robot.shoulderPort.setPosition(v3Hardware.shoulderPortScore);
                robot.shoulderStar.setPosition(v3Hardware.shoulderStarScore);
            }
            /*
            if (gamepad1.x) {
                robot.intake.setPower(v3Hardware.intakeSpeed);
            }
            else if (gamepad1.b) {
                robot.intake.setPower(-v3Hardware.intakeSpeed);
            }
            else{
                robot.intake.setPower(0);
            }

             */
            if(gamepad1.right_bumper){
                robot.flipper.setPosition(v3Hardware.flipUp);
            }
            if(gamepad1.left_bumper){
                robot.flipper.setPosition(v3Hardware.flipDown);
            }
            telemetry.addData("ticks", robot.aimer.getCurrentPosition());
            telemetry.addData("distance", robot.distanceTop.getDistance(DistanceUnit.MM));
            telemetry.update();
        }
    }
}
