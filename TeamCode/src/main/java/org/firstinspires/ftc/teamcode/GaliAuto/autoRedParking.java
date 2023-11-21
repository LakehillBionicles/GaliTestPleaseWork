package org.firstinspires.ftc.teamcode.GaliAuto;
import static org.firstinspires.ftc.teamcode.Vision.BlueColorProcessor.centerBlue;
import static org.firstinspires.ftc.teamcode.Vision.BlueColorProcessor.leftBlue;
import static org.firstinspires.ftc.teamcode.Vision.BlueColorProcessor.rightBlue;
import static org.firstinspires.ftc.teamcode.Vision.RedColorProcessor.centerRed;
import static org.firstinspires.ftc.teamcode.Vision.RedColorProcessor.centerRedRatio;
import static org.firstinspires.ftc.teamcode.Vision.RedColorProcessor.leftRed;
import static org.firstinspires.ftc.teamcode.Vision.RedColorProcessor.leftRedRatio;
import static org.firstinspires.ftc.teamcode.Vision.RedColorProcessor.rightRed;
import static org.firstinspires.ftc.teamcode.Vision.RedColorProcessor.pos;
import static org.firstinspires.ftc.teamcode.Vision.RedColorProcessor.rightRedRatio;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
@Autonomous
public class autoRedParking extends GaliAutobase{

    @Override
    public void runOpMode(){
        super.runOpMode();
        propDetection("red");
        //Objects.equals(propPos, "notSeen")&&
        while(!isStarted()){
            telemetry.addData("position",pos);
            telemetry.addData("leftBlue",leftRedRatio);
            telemetry.addData("centerBlue",centerRedRatio);
            telemetry.addData("rightBlue",rightRedRatio);
            telemetry.update();
        }

    }
}
