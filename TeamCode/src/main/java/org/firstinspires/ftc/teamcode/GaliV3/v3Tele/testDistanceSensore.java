package org.firstinspires.ftc.teamcode.GaliV3.v3Tele;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.GaliV3.v3Hardware;
@TeleOp
public class testDistanceSensore extends LinearOpMode {
    public v3Hardware robot = new v3Hardware();
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        telemetry.addData("distance", robot.distanceTop.getDistance(DistanceUnit.MM));
        telemetry.update();
        waitForStart();
        while (opModeIsActive()){
            telemetry.addData("distance", robot.distanceTop.getDistance(DistanceUnit.MM));
            telemetry.update();
        }
        //robot.extendyBoi.setPosition(v3Hardware.extendyBoiRetract);
        //robot.elbow.setPosition(v3Hardware.elbowNorminal);
        //robot.shoulderStar.setPosition(v3Hardware.shoulderStarDown);
        //robot.shoulderPort.setPosition(v3Hardware.shoulderPortDown);
        //robot.wrist.setPosition(v3Hardware.wristDown);
    }
}