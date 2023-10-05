package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;

public class DriveSubsystem extends SubsystemBase {
    private final DcMotor fpd, bpd, fsd, bsd;

    public DriveSubsystem(DcMotor fpd, DcMotor bpd, DcMotor fsd, DcMotor bsd){
        this.fpd = fpd; this.bpd = bpd; this.fsd = fsd; this.bsd = bsd;
    }

    public void setDrivePowerTele(double drivePower, double strafePower, double rotPower){
        fpd.setPower(drivePower - strafePower + rotPower);
        bpd.setPower(drivePower + strafePower + rotPower);
        fsd.setPower(drivePower + strafePower - rotPower);
        bsd.setPower(drivePower - strafePower - rotPower);
    }

    public void resetDrive(){
        fpd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bpd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fsd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bsd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fpd.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bpd.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fsd.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bsd.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
