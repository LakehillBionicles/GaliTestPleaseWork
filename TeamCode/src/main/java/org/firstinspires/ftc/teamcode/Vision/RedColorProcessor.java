package org.firstinspires.ftc.teamcode.Vision;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import static org.firstinspires.ftc.teamcode.GaliV3.v3Auto.v3autoBase.robotPosition;

import java.util.Objects;

@Config

public class RedColorProcessor extends OpenCvPipeline {
    public static int leftPointx1 = 20;
    public static int leftPointy1 = 50;
    public static int leftPointx2 = 100;
    public static int  leftPointy2 = 110;
    public static int CenterPointx1 = 90;
    public static int CenterPointy1 = 60;
    public static int CenterPointx2 = 150;
    public static int CenterPointy2 = 100;
    public static int rightPointx1 = 250;
    public static int rightPointy1 = 55;
    public static int rightPointx2 = 320;
    public static int rightPointy2 = 115;
    public static String pos = "notSeen";
    public static double leftRedRatio = 0;
    public static double centerRedRatio = 0;
    public static double rightRedRatio = 0;
    public static double redTolerance = 0.51;
    @Override
    public Mat processFrame(Mat input) {
        if(Objects.equals(robotPosition, "far")){
            CenterPointx1 = 150;
            CenterPointy1 = 40;
            CenterPointx2 = 220;
            CenterPointy2 = 100;
        }
        Point rightLeft = new Point(rightPointx1, rightPointy1);
        Point rightRight = new Point(rightPointx2, rightPointy2);
        Point centerLeft = new Point(CenterPointx1, CenterPointy1);
        Point centerRight = new Point(CenterPointx2, CenterPointy2);
        Point leftLeft = new Point(leftPointx1, leftPointy1);
        Point leftRight = new Point(leftPointx2, leftPointy2);
        Mat matRight = input.submat(new Rect(rightLeft, rightRight));
        Mat matCenter = input.submat(new Rect(centerLeft, centerRight));
        Mat matLeft = input.submat(new Rect(leftLeft, leftRight));
        Imgproc.rectangle(input,new Rect(rightLeft,rightRight), new Scalar(0, 255, 0));
        Imgproc.rectangle(input,new Rect(centerLeft, centerRight), new Scalar(0, 255, 0));
        Imgproc.rectangle(input,new Rect(leftLeft, leftRight), new Scalar(0, 255, 0));
        rightRedRatio = (Core.sumElems(matRight).val[0]/(matRight.width()*matRight.height())/((Core.sumElems(matRight).val[1]/(matRight.width()*matRight.height()))+(Core.sumElems(matRight).val[2]/(matRight.width()*matRight.height()))));
        centerRedRatio = (Core.sumElems(matCenter).val[0]/(matCenter.width()*matCenter.height())/((Core.sumElems(matCenter).val[1]/(matCenter.width()*matCenter.height()))+(Core.sumElems(matCenter).val[2]/(matCenter.width()*matCenter.height()))));
        leftRedRatio = (Core.sumElems(matLeft).val[0]/(matLeft.width()*matLeft.height())/((Core.sumElems(matLeft).val[1]/(matLeft.width()*matLeft.height()))+(Core.sumElems(matLeft).val[2]/(matLeft.width()*matLeft.height()))));
        /*
        if (leftBlueRatio > centerBlueRatio) {
            if (leftBlueRatio > rightBlueRatio) {
                pos = "left";
                Imgproc.rectangle(input, new Rect(leftLeft, leftRight), new Scalar(0, 0, 255));

            } else {
                pos = "right";
                Imgproc.rectangle(input, new Rect(rightLeft, rightRight), new Scalar(0, 0, 255));

            }
        } else {
            if (centerBlueRatio > rightBlueRatio) {
                pos = "center";
                Imgproc.rectangle(input, new Rect(centerLeft, centerRight), new Scalar(0, 0, 255));

            } else {
                pos = "right";
                Imgproc.rectangle(input, new Rect(rightLeft, rightRight), new Scalar(0, 0, 255));

            }
        }

         */
        if(leftRedRatio>redTolerance&& leftRedRatio>centerRedRatio){
            pos = "left";
            Imgproc.rectangle(input, new Rect(rightLeft, rightRight), new Scalar(255, 0, 0));
        }
        else if(centerRedRatio>redTolerance&&centerRedRatio>leftRedRatio){
            pos = "center";
            Imgproc.rectangle(input, new Rect(centerLeft, centerRight), new Scalar(255, 0, 0));
        }
        else{
            pos = "right";
            Imgproc.rectangle(input, new Rect(centerLeft, centerRight), new Scalar(255, 255, 255));
            Imgproc.rectangle(input, new Rect(rightLeft, rightRight), new Scalar(255, 255, 255));
        }
        matRight.release();
        matCenter.release();
        matLeft.release();
        return input;
    }
}
