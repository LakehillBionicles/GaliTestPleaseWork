package org.firstinspires.ftc.teamcode.GaliAuto;

import static org.firstinspires.ftc.teamcode.Vision.BlueColorProcessor.centerBlueRatio;
import static org.firstinspires.ftc.teamcode.Vision.BlueColorProcessor.leftBlueRatio;
import static org.firstinspires.ftc.teamcode.Vision.BlueColorProcessor.rightBlueRatio;
import static org.firstinspires.ftc.teamcode.Vision.BlueColorProcessor.pos;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
@Autonomous
public class autoBlueParking extends GaliAutobase{

    @Override
    public void runOpMode(){
        robot.init(hardwareMap);
        super.runOpMode();
        cameraStartup("Webcam 1");
        propDetection("blue");
        //Objects.equals(propPos, "notSeen")&&
        while(!isStarted()){
            telemetry.addData("position", pos);
            telemetry.addData("leftBlue",leftBlueRatio);
            telemetry.addData("centerBlue",centerBlueRatio);
            telemetry.addData("rightBlue",rightBlueRatio);

            telemetry.update();
        }

    }
}
