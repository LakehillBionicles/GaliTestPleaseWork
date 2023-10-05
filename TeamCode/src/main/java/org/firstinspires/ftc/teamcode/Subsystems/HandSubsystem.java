package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

public class HandSubsystem extends SubsystemBase {
    private final Servo handPort;
    private final Servo handStar;

    public enum HandPos {
        OPEN(0.0, 0.0), CLOSED(0.0, 0.0);

        public final double portPos, starPos;

        HandPos(double portPos, double starPos){this.portPos = portPos; this.starPos = starPos;}

        public double getStarPos(){return starPos;}
        public double getPortPos(){return portPos;}
    }

    public HandSubsystem(Servo servo1, Servo servo2){
        this.handPort = servo1;
        this.handStar = servo2;
    }

    public Command grabBoth() { return new InstantCommand(() -> setHandPosBoth(HandPos.CLOSED));}
    public Command openBoth() { return new InstantCommand(() -> setHandPosBoth(HandPos.OPEN));}
    public Command closePort() { return new InstantCommand(() -> setHandPosPort(HandPos.CLOSED));}
    public Command openPort() { return new InstantCommand(() -> setHandPosPort(HandPos.OPEN));}
    public Command closeStar() { return new InstantCommand(() -> setHandPosStar(HandPos.CLOSED));}
    public Command openStar() { return new InstantCommand(() -> setHandPosStar(HandPos.OPEN));}
    public void setHandPosBoth(HandPos target){
        setHandPosPort(target);
        setHandPosStar(target);
    }
    public void setHandPosPort(HandPos targetPos){
        handPort.setPosition(targetPos.getPortPos());
    }
    public void setHandPosStar(HandPos targetPos){
        handStar.setPosition(targetPos.getStarPos());
    }

}
