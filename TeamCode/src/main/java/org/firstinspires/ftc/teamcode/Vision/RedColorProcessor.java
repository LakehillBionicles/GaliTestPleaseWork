package org.firstinspires.ftc.teamcode.Vision;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

@Config

public class RedColorProcessor extends OpenCvPipeline {
    public static int rightPointx1 = 0;
    public static int rightPointy1 = 135;
    public static int rightPointx2 = 100;
    public static int rightPointy2 = 190;
    public static int CenterPointx1 = 100;
    public static int CenterPointy1 = 135;
    public static int CenterPointx2 = 250;
    public static int CenterPointy2 = 175;
    public static String pos = "notSeen";
    public static Scalar leftTotal = Scalar.all(0);
    public static Scalar centerTotal = Scalar.all(0);
    public static Scalar rightTotal = Scalar.all(0);
    public static double centerRed = 0;
    public static double rightRed = 0;
    public static double centerRedRatio = 0;
    public static double rightRedRatio = 0;
    public static double redTolerance = 0.5;
    @Override
    public Mat processFrame(Mat input) {
        Point rightLeft = new Point(rightPointx1, rightPointy1);
        Point rightRight = new Point(rightPointx2, rightPointy2);
        Point centerLeft = new Point(CenterPointx1, CenterPointy1);
        Point centerRight = new Point(CenterPointx2, CenterPointy2);
        Mat matRight = input.submat(new Rect(rightLeft, rightRight));
        Mat matCenter = input.submat(new Rect(centerLeft, centerRight));
        Imgproc.rectangle(input,new Rect(rightLeft,rightRight), new Scalar(0, 255, 0));
        Imgproc.rectangle(input,new Rect(centerLeft, centerRight), new Scalar(0, 255, 0));
        rightTotal = Core.sumElems(matRight);
        centerTotal = Core.sumElems(matCenter);
        rightRed = Core.sumElems(matRight).val[0]/(matRight.width()*matRight.height());
        centerRed = Core.sumElems(matCenter).val[0]/(matCenter.width()*matCenter.height());
        rightRedRatio = (Core.sumElems(matRight).val[0]/(matRight.width()*matRight.height())/((Core.sumElems(matRight).val[1]/(matRight.width()*matRight.height()))+(Core.sumElems(matRight).val[2]/(matRight.width()*matRight.height()))));
        centerRedRatio = (Core.sumElems(matCenter).val[0]/(matCenter.width()*matCenter.height())/((Core.sumElems(matCenter).val[1]/(matCenter.width()*matCenter.height()))+(Core.sumElems(matCenter).val[2]/(matCenter.width()*matCenter.height()))));
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
        if(rightRedRatio>redTolerance){
            pos = "left";
            Imgproc.rectangle(input, new Rect(rightLeft, rightRight), new Scalar(255, 0, 0));
        }
        else if(centerRedRatio>redTolerance){
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
        return input;
    }
}
