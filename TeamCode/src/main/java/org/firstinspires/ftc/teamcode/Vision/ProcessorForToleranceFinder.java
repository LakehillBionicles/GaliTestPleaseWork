package org.firstinspires.ftc.teamcode.Vision;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
public class ProcessorForToleranceFinder extends OpenCvPipeline{

    /*
            YELLOW  = Parking Left
            CYAN    = Parking Middle
            MAGENTA = Parking Right
             */
    public enum ParkingPosition {
        NOTSEEN, ONE, TWO, THREE, FOUR, FIVE, SIX, SEVEN, EIGHT, NINE, TEN, ELEVEN, TWELVE,
        THIRTEEN, FOURTEEN, FIFTEEN, SIXTEEN, SEVENTEEN, EIGHTEEN, NINETEEN, TWENTY, TWENTYONE,
        TWENTYTWO, TWENTYTHREE, TWENTYFOUR
    }

    public enum RedParkingPosition {
        NOTSEEN, ONE, TWO, THREE, FOUR, FIVE, SIX, SEVEN, EIGHT, NINE, TEN, ELEVEN, TWELVE
    }

    // Width and height for the bounding box
    // Color definitions
    private final Scalar
            BLUE = new Scalar(0, 0, 255),
            GREEN = new Scalar(0, 255, 0),
            CYAN = new Scalar(0, 255, 255),
            RED = new Scalar(255, 0, 0),

    WHITE = new Scalar(255, 255, 255),

    YELLOW = new Scalar(255, 255, 0);

    // Anchor point definitions
    // Running variable storing the parking position
    private static volatile ColorProcessor.ParkingPosition bluePosition = ColorProcessor.ParkingPosition.ONE;
    private static volatile ColorProcessor.RedParkingPosition redPosition = ColorProcessor.RedParkingPosition.NOTSEEN;
    int anchorWidth = 4;

    int bottomHeight = 1;
    int boxWidth = 20;
    static int addedBlueBarPositions = 0;
    static int positionOfBlueObject = 0;
    static int numberOfBlueBars = 0;
    static int addedRedBarPositions = 0;
    static int positionOfRedObject = 0;
    static int numberOfRedBars = 0;

    static int addedYellowBarPositions = 0;
    static int positionOfYellowObject = 0;
    static int numberOfYellowBars = 0;

    static double blueDistance = 0;
    static double redDistance = 0;
    static double widthOfInput;

    static double heightOfInput;

    static double redTolerance = 0.6;

    static double blueTolerance = 0.6;

    static double yellowTolerance = 4;//Yellow tolerance needs to be roughly twice as high as red and blue tolerance
    public static Scalar telemetryRGBValues;

    static double red = 0;
    static double blue = 0;
    static double green = 0;
    public Mat processFrame(Mat input) {
        widthOfInput = input.width();
        heightOfInput = input.height() / 2;
        addedBlueBarPositions = 0;
        numberOfBlueBars = 0;
        addedRedBarPositions = 0;
        positionOfRedObject = 0;
        numberOfRedBars = 0;
        addedYellowBarPositions = 0;
        positionOfYellowObject = 0;
        numberOfYellowBars = 0;

        Scalar colors;
        Point BarPoint1 = new Point((widthOfInput/2)-20,(heightOfInput)+20);
        Point BarPoint2 = new Point((widthOfInput/2)+ 20, (heightOfInput/2)-20);
        Mat MatArea1 = input.submat(new Rect(BarPoint1, BarPoint2));
        colors = Core.sumElems(MatArea1);
        red = colors.val[0];
        blue = colors.val[1];
        green = colors.val[2];
        MatArea1.release();


        // Get the submat frame, and then sum all the values
        //Used for telemetry will remove
        Imgproc.rectangle(
                input,
                BarPoint1,
                BarPoint2,
                GREEN,
                2
        );
        return input;
    }
    public static double getBlue(){
        return blue;
    }public static double getRed(){
        return red;
    }public static double getGreen(){
        return green;
    }
    //This Doesn't work yet
}

