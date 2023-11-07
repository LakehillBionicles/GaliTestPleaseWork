package org.firstinspires.ftc.teamcode.Testing;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
@Config
public class TestMotors extends OpMode {
    public static double motorPower = 0;
    public static double intakePower = 0;
    public static double armPower = 0;
    public DcMotor intake, fpd, bpd, fsd, bsd, SOW, armPort, armStar;

    public void init(){
        intake = hardwareMap.get(DcMotor.class, "intake");
        fpd = hardwareMap.get(DcMotor.class, "fpd");
        bpd = hardwareMap.get(DcMotor.class, "bpd");
        fsd = hardwareMap.get(DcMotor.class, "fsd");
        bsd = hardwareMap.get(DcMotor.class, "bsd");
        SOW = hardwareMap.get(DcMotor.class, "SOW");
        armPort = hardwareMap.get(DcMotor.class, "armPort");
        armStar = hardwareMap.get(DcMotor.class, "armStar");

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    public void loop(){
        intake.setPower(intakePower);

        fpd.setPower(motorPower);
        bpd.setPower(motorPower);
        fsd.setPower(motorPower);
        bsd.setPower(motorPower);

        armPort.setPower(armPower);
        armStar.setPower(armPower);

        telemetry.addData("BOW", bpd.getCurrentPosition());
        telemetry.addData("POW", fpd.getCurrentPosition());
        telemetry.addData("SOW", SOW.getCurrentPosition());
        telemetry.update();
    }
}
