package testing;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class SkystoneDetector extends OpenCvPipeline {
    private Mat workingMatrix = new Mat();
    public SkystoneDetector(){

    }

    @Override
    public final Mat processFrame(Mat input){
        input.copyTo(workingMatrix);

        if(workingMatrix.empty()){
            return input;
        }

        Imgproc.cvtColor(workingMatrix, workingMatrix, Imgproc.COLOR_RGB2YCrCb);

        Mat matLeft = workingMatrix.submat(120, 150, 10, 50);
        Mat matCenter = workingMatrix.submat(120, 150, 80, 120);
        Mat matRight = workingMatrix.submat(120, 150, 150, 190);

        Imgproc.rectangle(workingMatrix, new Rect(10, 120, 40, 30), new Scalar(0, 255, 0));
        Imgproc.rectangle(workingMatrix, new Rect(80, 120, 40, 30), new Scalar(0, 255, 0));
        Imgproc.rectangle(workingMatrix, new Rect(150, 120, 40, 30), new Scalar(0, 255, 0));

        double leftSum = Core.sumElems(matLeft).val[2];
        double centerSum = Core.sumElems(matCenter).val[2];
        double rightSum = Core.sumElems(matRight).val[2];

        if(leftSum > centerSum){
            if (leftSum > rightSum) {
                //skystone is left
            }
            else {
                //skystone is right
            }
        }

        else {
            if (centerSum > rightSum) {
                //skystone is center
            }
            else {
                //skystone is right
            }
        }

        return workingMatrix;
    }
}
