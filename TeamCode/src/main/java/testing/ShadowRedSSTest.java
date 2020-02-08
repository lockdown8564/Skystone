package testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

import ftclib.FtcChoiceMenu;
import ftclib.FtcMenu;
import ftclib.FtcValueMenu;
import hallib.HalDashboard;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

/**
 * red stone auto test
 * created: 2/7/20
 * last updated: 2/7/207
 */

@Autonomous(name = "red ss test", group = "test")
public class ShadowRedSSTest extends LinearOpMode{
    private ShadowTestHardware robot = new ShadowTestHardware();
    private OpenCvCamera webcam;
    private SSDetector detector;
    private Skystone skystone = Skystone.RIGHT;

    private enum Skystone{
        LEFT,
        MIDDLE,
        RIGHT
    }

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        detector = new SSDetector();

        webcam.openCameraDevice();
        webcam.setPipeline(detector);
        webcam.startStreaming(1920, 1080, OpenCvCameraRotation.UPRIGHT);

        while(!isStarted()) {
            skystone = detector.skystone;
            telemetry.addData("skystone:", skystone);
            telemetry.update();
        }

        waitForStart();

        switch(skystone){
            case LEFT:{

                break;
            }

            case MIDDLE:{

                break;
            }

            case RIGHT:{

                break;
            }
        }

    }

    private void encoderDrive(double speed, double leftInches, double rightInches){
        int LEFT_TARGET, RIGHT_TARGET;
        if(opModeIsActive()){
            LEFT_TARGET = (int)(leftInches*robot.getCPI()) + robot.frontLeft.getCurrentPosition();
            RIGHT_TARGET = (int)(rightInches*robot.getCPI()) + robot.frontRight.getCurrentPosition();

            robot.driveSetTarget(LEFT_TARGET,RIGHT_TARGET);
            robot.driveSetMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.driveSetPowerAll(Math.abs(speed));
            while(robot.driveIsBusy() && opModeIsActive()){
            }

            robot.stopMotors();
            robot.driveSetMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    private void encoderDriveIntake(double speed, double leftInches, double rightInches, double direction){
        int LEFT_TARGET, RIGHT_TARGET;
        if(opModeIsActive()){
            LEFT_TARGET = (int)(leftInches*robot.getCPI()) + robot.frontLeft.getCurrentPosition();
            RIGHT_TARGET = (int)(rightInches*robot.getCPI()) + robot.frontRight.getCurrentPosition();

            robot.driveSetTarget(LEFT_TARGET,RIGHT_TARGET);
            robot.driveSetMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.driveSetPowerAll(Math.abs(speed));
            while(robot.driveIsBusy() && opModeIsActive()){
                robot.intakeSetPower(direction);
            }

            robot.stopMotors();
            robot.driveSetMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    private void turnLeftCurvy(final float TARGET_ANGLE, double power){
        while(opModeIsActive()){
            float currentAngle = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            while(currentAngle>=-TARGET_ANGLE){
                currentAngle = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                robot.driveSetPower(power*4, power, power*4, power);
            }
            robot.stopMotors();
            break;
        }
    }

    private void turnRight(final float TARGET_ANGLE, double power){
        while(opModeIsActive()){
            float currentAngle = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            while(currentAngle>=-TARGET_ANGLE){
                currentAngle = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                robot.driveSetPower(-power, power, -power, power);
            }
            robot.stopMotors();
            break;
        }
    }

    private void turnLeftPivot(final float TARGET_ANGLE, double power){
        while(opModeIsActive()){
            float currentAngle = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            while(currentAngle<=TARGET_ANGLE){
                currentAngle = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                robot.driveSetPower(power, 0, power, 0);
                telemetry.addData("Heading:",currentAngle);
                telemetry.update();
            }
            robot.stopMotors();
            break;
        }
    }

    private void turnRightPivot(final float TARGET_ANGLE, double power){
        while(opModeIsActive()){
            float currentAngle = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            while(currentAngle>=-TARGET_ANGLE){
                currentAngle = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                robot.driveSetPower(0, power, 0, power);
            }
            robot.stopMotors();
            break;
        }
    }

    private void turnLeft(final float TARGET_ANGLE, double power){
        while(opModeIsActive()){
            float currentAngle = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            while(currentAngle<=TARGET_ANGLE){
                currentAngle = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                robot.driveSetPower(power, -power, power, -power);
                telemetry.addData("Heading:",currentAngle);
                telemetry.update();
            }
            robot.stopMotors();
            break;
        }
    }

    private void encImuDrive(double speed, double distance, double angle){
        int LEFT_TARGET, RIGHT_TARGET;
        double error;
        double steer;
        double leftSpeed, rightSpeed;
        double max;

        if(opModeIsActive()){
            LEFT_TARGET = (int)(distance*robot.getCPI()) + robot.frontLeft.getCurrentPosition();
            RIGHT_TARGET = (int)(distance*robot.getCPI()) + robot.frontRight.getCurrentPosition();

            robot.driveSetTarget(LEFT_TARGET,RIGHT_TARGET);
            robot.driveSetMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.driveSetPowerAll(Math.abs(speed));

            while(robot.driveIsBusy() && opModeIsActive()){
                error = robot.getError(angle);
                steer = robot.getSteer(error, robot.P_DRIVE_COEFF);

                if (distance < 0) {
                    steer *= -1.0;
                }

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));

                if(max > 1.0){
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                robot.driveSetPower(leftSpeed, rightSpeed, leftSpeed, rightSpeed);

            }

            robot.stopMotors();
            robot.driveSetMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    // - is right, + is left
    private void imuStrafe(double speed, double distance, double angle){
        int flTarget, frTarget, blTarget, brTarget;
        double error;
        double steer;
        double leftSpeed, rightSpeed;
        double max;

        if(opModeIsActive()){
            flTarget = (robot.frontLeft.getCurrentPosition()) + (int)(distance*robot.getCPI());
            frTarget = (robot.frontRight.getCurrentPosition()) - (int)(distance*robot.getCPI());
            blTarget = (robot.backLeft.getCurrentPosition()) - (int)(distance*robot.getCPI());
            brTarget = (robot.backRight.getCurrentPosition()) + (int)(distance*robot.getCPI());

            robot.driveSetTargetInd(flTarget, frTarget, blTarget, brTarget);
            robot.driveSetMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.driveSetPowerAll(Math.abs(speed));

            while(robot.driveIsBusy() && opModeIsActive()){
                error = robot.getError(angle);
                steer = robot.getSteer(error, robot.P_DRIVE_COEFF);

                if (distance < 0) {
                    steer *= -1.0;
                }

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));

                if(max > 1.0){
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                robot.driveSetPower(leftSpeed, rightSpeed, leftSpeed, rightSpeed);

            }

            robot.stopMotors();
            robot.driveSetMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    static class SSDetector extends OpenCvPipeline {
        private Mat workingMatrix = new Mat();
        Skystone skystone = Skystone.RIGHT;

        @Override
        public final Mat processFrame(Mat input) {
            input.copyTo(workingMatrix);

            if (workingMatrix.empty()) {
                return input;
            }

            Imgproc.cvtColor(workingMatrix, workingMatrix, Imgproc.COLOR_RGB2YCrCb);

            Mat matLeft = workingMatrix.submat(600, 700, 400, 640);
            Mat matCenter = workingMatrix.submat(600, 700, 820, 1060);
            Mat matRight = workingMatrix.submat(600, 700, 1240, 1480);

            Imgproc.rectangle(workingMatrix, new Rect(400, 600, 170, 120), new Scalar(0, 255, 0));
            Imgproc.rectangle(workingMatrix, new Rect(820, 600, 170, 120), new Scalar(0, 255, 0));
            Imgproc.rectangle(workingMatrix, new Rect(1240, 600, 170, 120), new Scalar(0, 255, 0));

            double leftSum = Core.sumElems(matLeft).val[2];
            double centerSum = Core.sumElems(matCenter).val[2];
            double rightSum = Core.sumElems(matRight).val[2];

            if (leftSum > centerSum) {
                if (leftSum > rightSum) {
                    //skystone is left
                    skystone = Skystone.LEFT;
                } else {
                    //skystone is right
                    skystone = Skystone.RIGHT;
                }
            } else {
                if (centerSum > rightSum) {
                    //skystone is center
                    skystone = Skystone.MIDDLE;
                } else {
                    //skystone is right
                    skystone = Skystone.RIGHT;
                }
            }

            return workingMatrix;
        }
    }
}
