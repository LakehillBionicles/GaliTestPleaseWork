package org.firstinspires.ftc.teamcode.Tele;

import static org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem.ArmPos.*;
import static org.firstinspires.ftc.teamcode.Subsystems.HandSubsystem.HandPos.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.GaliHardware;
import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem.ArmPos;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.HandSubsystem;

@TeleOp
public class GaliTeleCleaner extends LinearOpMode {
    GaliHardware robot = new GaliHardware();

    ArmPos armPos = DOWN_FRONT;

    ArmSubsystem galiArm = new ArmSubsystem(robot.elbow, robot.shoulder, robot.wrist);
    HandSubsystem galiHand = new HandSubsystem(robot.handPort, robot.handStar);
    DriveSubsystem galiDrive = new DriveSubsystem(robot.fpd, robot.bpd, robot.fsd, robot.bsd);

    //GamepadEx baseController = new GamepadEx(gamepad1);
    //GamepadEx armController = new GamepadEx(gamepad2);
    public void runOpMode() {
        robot.init(hardwareMap);
        galiArm.resetArm();

        waitForStart();

        while (opModeIsActive()) {
            //galiDrive.setDrivePowerTele(-baseController.getLeftY(), baseController.getLeftX(), baseController.getRightX());

            galiArm.setArmPos(getArmPos());

            //galiHand.setHandPosPort(getHandPosPort());
            //galiHand.setHandPosStar(getHandPosStar());
        }
    }
    public ArmPos getArmPos(){
        if(gamepad2.dpad_up){
            armPos = HIGH_FRONT;
        }
        if(gamepad2.dpad_right){
            armPos = MID_FRONT;
        }
        if(gamepad2.dpad_left){
            armPos = LOW_FRONT;
        }
        if(gamepad2.dpad_down){
            armPos = DOWN_FRONT;
        }
        /*if(armControlButton(DPAD_DOWN).get()){ armPos = DOWN_FRONT;}
        if(armControlButton(DPAD_LEFT).get()){ armPos = LOW_FRONT;}
        if(armControlButton(DPAD_RIGHT).get()){ armPos = MID_FRONT;}
        if(armControlButton(DPAD_UP).get()){ armPos = HIGH_FRONT;}*/

        return armPos;
    }

    /*public HandPos getHandPosPort(){
        HandPos handPos = null;
        if(armController.getButton(LEFT_BUMPER)){
            handPos = CLOSED;
        } else if (armController.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)>0){
            handPos = OPEN;
        }

        return handPos;
    }*/

    /*public HandPos getHandPosStar(){
        HandPos handPos = null;
        if(armController.getButton(RIGHT_BUMPER)){
            handPos = CLOSED;
        } else if (armController.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)>0){
            handPos = OPEN;
        }
        return handPos;
    }*/

    //public GamepadButton armControlButton(GamepadKeys.Button button){ return armController.getGamepadButton(button); }
}
