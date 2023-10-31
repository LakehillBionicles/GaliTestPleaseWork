package org.firstinspires.ftc.teamcode.Vision;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class RedColorProcessor extends OpenCvPipeline {
    public static int leftPointx1 = 10;
    public static int leftPointy1 = 10;
    public static int leftPointx2 = 90;
    public static int leftPointy2 = 80;
    public static int CenterPointx1 = 100;
    public static int CenterPointy1 = 10;
    public static int CenterPointx2 = 180;
    public static int CenterPointy2 = 80;
    public static int RightPointx1 = 190;
    public static int RightPointy1 = 10;
    public static int RightPointx2 = 270;
    public static int RightPointy2 = 80;

    public String pos = "notSeen";


    private Mat workingMat = new Mat();
    public RedColorProcessor(){
    }

    @Override
    public Mat processFrame(Mat input) {
        input.copyTo(workingMat);
        if(workingMat.empty()) {
            return input;
        }
        Mat matLeft = workingMat.submat(leftPointx1,leftPointx2, leftPointy1, leftPointy2);
        Mat matCenter = workingMat.submat(CenterPointx1,CenterPointx2, CenterPointy1, CenterPointy2);
        Mat matRight = workingMat.submat(190,270, 20, 80);
        Imgproc.rectangle(workingMat,new Rect(leftPointx1, leftPointy1, leftPointx2, leftPointy2), new Scalar(0, 255, 0));
        Imgproc.rectangle(workingMat,new Rect(CenterPointx1, CenterPointy1, CenterPointx2, CenterPointy2), new Scalar(0, 255, 0));
        Imgproc.rectangle(workingMat,new Rect(RightPointx1, RightPointy1, RightPointx2, RightPointy2), new Scalar(0, 255, 0));

        double leftTotal = Core.sumElems(matLeft).val[1];
        double centerTotal = Core.sumElems(matCenter).val[1];
        double rightTotal = Core.sumElems(matRight).val[1];

        if(leftTotal>centerTotal){
            if(leftTotal>rightTotal){
                pos = "left";
            }else{
                pos = "center";
            }
        }else{
            if(centerTotal>rightTotal){
                pos = "center";
            }else{
                pos = "right";
            }
        }
        return workingMat;
    }
}
