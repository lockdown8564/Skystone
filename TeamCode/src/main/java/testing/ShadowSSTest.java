package testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

/**
 * This program was created to test OpenCv for detecting a Skystone.
 * It uses a class called SSDetector that looks at 3 specific spots
 * on the screen and returns thehues in YCbCr. Next, it processes the
 * images and determines where the skystone is based on the hues. This was
 * the first OpenCv test we had this season.
 *
 * @author William Trang
 * @version 2.0 2/9/20
 */

@Disabled
@Autonomous(name = "ss test 1", group = "test")
public class ShadowSSTest extends LinearOpMode {
    private OpenCvCamera webcam;

    @Override
    public void runOpMode(){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        webcam.openCameraDevice();
        webcam.setPipeline(new SSDetector());
        webcam.startStreaming(1920, 1080, OpenCvCameraRotation.UPRIGHT);
    }

    static class SSDetector extends OpenCvPipeline {
        private Mat workingMatrix = new Mat();

        @Override
        public final Mat processFrame(Mat input){
            input.copyTo(workingMatrix);

            if(workingMatrix.empty()){
                return input;
            }

            Imgproc.cvtColor(workingMatrix, workingMatrix, Imgproc.COLOR_RGB2YCrCb);

            Mat matLeft = workingMatrix.submat(540, 700, 100, 300);
            Mat matCenter = workingMatrix.submat(540, 700, 450, 650);
            Mat matRight = workingMatrix.submat(540, 700, 800, 1000);

            Imgproc.rectangle(workingMatrix, new Rect(100, 540, 200, 150), new Scalar(0, 255, 0));
            Imgproc.rectangle(workingMatrix, new Rect(450, 540, 200, 150), new Scalar(0, 255, 0));
            Imgproc.rectangle(workingMatrix, new Rect(800, 540, 200, 150), new Scalar(0, 255, 0));

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
}
