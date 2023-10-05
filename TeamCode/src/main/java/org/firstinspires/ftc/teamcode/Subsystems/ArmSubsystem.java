package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class ArmSubsystem extends SubsystemBase {
    private final DcMotor elbow, shoulder;
    private final Servo wrist;

    public enum ArmPos {
        DOWN_FRONT(0, 0, 0.0),
        LOW_FRONT(350, 380, 0.0),
        MID_FRONT(450, 270, 0.0),
        HIGH_FRONT(960, 525, 0.0),
        LOW_BACK(0, 0, 0.0),
        MID_BACK(0, 0, 0.0),
        HIGH_BACK(0, 0, 0.0);

        public final int elbowPos, shoulderPos;
        public final double wristPos;

        ArmPos(int elbowPos, int shoulderPos, double wristPos){this.elbowPos = elbowPos; this.shoulderPos = shoulderPos; this.wristPos = wristPos;}

        public int getElbowPos(){return elbowPos;}
        public int getShoulderPos(){return shoulderPos;}
        public double getWristPos(){return wristPos;}
    }

    public ArmSubsystem(DcMotor motor1, DcMotor motor2, Servo servo){
        this.elbow = motor1; this.shoulder = motor2; this.wrist = servo;
    }

    public Command setArmPos(ArmPos targetPos){
        return new InstantCommand(() -> armToPosition(targetPos, .25));
    }
    public void armToPosition(ArmPos targetPos, double power){
        setArmTarget(targetPos.getElbowPos(), targetPos.getShoulderPos());
        setArmMode(DcMotor.RunMode.RUN_TO_POSITION);
        setArmPower(power);
        wrist.setPosition(targetPos.getWristPos());
    }

    public void setArmMode(DcMotor.RunMode runMode){
        elbow.setMode(runMode);
        shoulder.setMode(runMode);
    }

    public void setArmTarget(int targetElbow, int targetShoulder){
        elbow.setTargetPosition(targetElbow);
        shoulder.setTargetPosition(targetShoulder);
    }

    public void resetArm(){
        elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        elbow.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shoulder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setArmPower(double power) {
        elbow.setPower(power);
        shoulder.setPower(power);
    }

}