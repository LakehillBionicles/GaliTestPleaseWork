package org.firstinspires.ftc.teamcode.Vision;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvPipeline;

public class OpenCVImageClassificationPipeline extends OpenCvPipeline {
    @Override
    public Mat processFrame(Mat input)
    {
        return input;
    }

    /*
    private Mat workingMatrix = new Mat();
    public OpenCVImageClassificationPipeline(){

    }
    @Override
    public final Mat processFrame(Mat input){
        input.copyTo(workingMatrix);

        if (workingMatrix.empty()){
            return input;
        }
        Imgproc.cvtColor(workingMatrix, workingMatrix, Imgproc.COLOR_RGB2YCrCb);
    }

     */
}
