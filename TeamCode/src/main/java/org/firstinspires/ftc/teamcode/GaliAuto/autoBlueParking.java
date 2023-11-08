package org.firstinspires.ftc.teamcode.GaliAuto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class autoBlueParking extends GaliAutobase {

    @Override
    public void runOpMode(){
        super.runOpMode();
        propDetection("blue");
        //Objects.equals(propPos, "notSeen")&&
        while(!isStarted()){
            telemetry.addData("position",propPos);
        }
    }
}
