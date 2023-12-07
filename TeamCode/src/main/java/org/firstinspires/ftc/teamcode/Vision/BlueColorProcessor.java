package org.firstinspires.ftc.teamcode.Vision;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import android.graphics.BlendMode;
import android.graphics.Interpolator;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
@Config

public class BlueColorProcessor extends OpenCvPipeline {
    public static int leftPointx1 = 0;
    public static int leftPointy1 = 135;
    public static int leftPointx2 = 100;
    public static int leftPointy2 = 190;
    public static int CenterPointx1 = 100;
    public static int CenterPointy1 = 135;
    public static int CenterPointx2 = 220;
    public static int CenterPointy2 = 175;
    public static String pos = "notSeen";
    public static Scalar leftTotal = Scalar.all(0);
    public static Scalar centerTotal = Scalar.all(0);
    public static Scalar rightTotal = Scalar.all(0);
    public static double centerRed = 0;
    public static double centerGreen = 0;
    public static double leftBlue = 0;
    public static double centerBlue = 0;
    public static double rightBlue = 0;
    public static double leftBlueRatio = 0;
    public static double centerBlueRatio = 0;
    public static double rightBlueRatio = 0;
    public static double blueTolerance = 0.55;
    @Override
    public Mat processFrame(Mat input) {
        Point leftLeft = new Point(leftPointx1, leftPointy1);
        Point leftRight = new Point(leftPointx2, leftPointy2);
        Point centerLeft = new Point(CenterPointx1, CenterPointy1);
        Point centerRight = new Point(CenterPointx2, CenterPointy2);
        Mat matLeft = input.submat(new Rect(leftLeft, leftRight));
        Mat matCenter = input.submat(new Rect(centerLeft, centerRight));
        Imgproc.rectangle(input,new Rect(leftLeft,leftRight), new Scalar(0, 255, 0));
        Imgproc.rectangle(input,new Rect(centerLeft, centerRight), new Scalar(0, 255, 0));
        leftTotal = Core.sumElems(matLeft);
        centerTotal = Core.sumElems(matCenter);
        leftBlue = Core.sumElems(matLeft).val[2]/(matLeft.width()*matLeft.height());
        centerBlue = Core.sumElems(matCenter).val[2]/(matCenter.width()*matCenter.height());
        leftBlueRatio = (Core.sumElems(matLeft).val[2]/(matLeft.width()*matLeft.height())/((Core.sumElems(matLeft).val[1]/(matLeft.width()*matLeft.height()))+(Core.sumElems(matLeft).val[0]/(matLeft.width()*matLeft.height()))));
        centerBlueRatio = (Core.sumElems(matCenter).val[2]/(matCenter.width()*matCenter.height())/((Core.sumElems(matCenter).val[1]/(matCenter.width()*matCenter.height()))+(Core.sumElems(matCenter).val[0]/(matCenter.width()*matCenter.height()))));
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
        if(leftBlueRatio>blueTolerance){
            pos = "left";
            Imgproc.rectangle(input, new Rect(leftLeft, leftRight), new Scalar(0, 0, 255));
        }
        else if(centerBlueRatio>blueTolerance){
            pos = "center";
            Imgproc.rectangle(input, new Rect(centerLeft, centerRight), new Scalar(0, 0, 255));
        }
        else{
            pos = "right";
            Imgproc.rectangle(input, new Rect(centerLeft, centerRight), new Scalar(255, 255, 255));
            Imgproc.rectangle(input, new Rect(leftLeft, leftRight), new Scalar(255, 255, 255));
        }
        matLeft.release();
        matCenter.release();
        return input;
    }
}
