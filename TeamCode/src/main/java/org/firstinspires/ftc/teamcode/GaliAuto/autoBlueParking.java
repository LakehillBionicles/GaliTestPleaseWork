package org.firstinspires.ftc.teamcode.GaliAuto;

import static org.firstinspires.ftc.teamcode.Vision.BlueColorProcessor.centerBlue;
import static org.firstinspires.ftc.teamcode.Vision.BlueColorProcessor.centerGreen;
import static org.firstinspires.ftc.teamcode.Vision.BlueColorProcessor.centerRed;
import static org.firstinspires.ftc.teamcode.Vision.BlueColorProcessor.centerTotal;
import static org.firstinspires.ftc.teamcode.Vision.BlueColorProcessor.leftBlue;
import static org.firstinspires.ftc.teamcode.Vision.BlueColorProcessor.rightBlue;
import static org.firstinspires.ftc.teamcode.Vision.BlueColorProcessor.pos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class autoBlueParking extends GaliAutobase {

    @Override
    public void runOpMode(){
        super.runOpMode();
        propDetection("blue");
        //Objects.equals(propPos, "notSeen")&&
        while(!isStarted()){
            telemetry.addData("position",pos);
            telemetry.addData("leftBlue",leftBlue);
            telemetry.addData("centerBlue",centerBlue);
            telemetry.addData("rightBlue",rightBlue);
            telemetry.update();
        }

    }
}
