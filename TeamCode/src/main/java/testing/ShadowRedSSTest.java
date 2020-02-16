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
 * Autonomous that starts on the red side and uses OpenCv
 * to get a stone and also parks.
 *
 * @author William Trang
 * @version 2.2 2/15/20
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

        encoderDrive(0.5, -23.5, -23.5);
        turnRight(87, 0.4);

        switch(skystone){
            case LEFT:{
                encoderDrive(0.6, 15.5, 15.5);

                robot.strafe(1, 0.5);
                sleep(500);
                robot.stopMotors();

                turnRight(113, 0.4);
                encoderDriveIntake(0.5, 25, 25, -1);
                robot.intakeSetPower(-1);
                sleep(1000);
                robot.stopMotors();

                encoderDrive(0.6, -33, -33);
                turnLeft(-92, 0.3);
                encoderDrive(0.6, -50, -50);

                break;
            }

            case MIDDLE:{
                encoderDrive(0.6, 14, 14);
                turnRight(120, 0.4);
                encoderDriveIntake(0.6, 10, 10, -1);
                encoderDrive(0.6, -10, -10);
                turnLeft(-90, 0.3);
                encoderDrive(0.6, -36, -36);
                break;
            }

            case RIGHT:{
                encoderDrive(0.6, 12, 12);
                turnRight(120, 0.4);
                encoderDriveIntake(0.6, 10, 10, -1);
                encoderDrive(0.6, -10, -10);
                turnLeft(-90, 0.3);
                encoderDrive(0.6, -32, -32);
                break;
            }
        }

        webcam.stopStreaming();
        webcam.closeCameraDevice();

    }

    /**
     * Drive an inputted distance in inches using encoders and motor's
     * run to position mode.
     *
     * @param speed       the desired speed to move at
     * @param leftInches  distance to move the left wheels
     * @param rightInches distance to move the right wheels
     */
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

    /**
     * Basically the encoderDrive method but runs the intake while it is moving.
     * Moves the robot a desired distance based on input.
     *
     * @param speed       the desired speed to move at
     * @param leftInches  distance to move the left wheels
     * @param rightInches distance to move the right wheels
     * @param direction   direction to move the intake in (- is in, + is out)
     */
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

    /**
     * Creates an arc moving left of the robot. Uses the REV Expansion Hub's
     * built in IMU.
     *
     * @param TARGET_ANGLE desired angle to turn to
     * @param power        desired power to turn at
     */
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

    /**
     * Turn right and stop at a desired inputted angle. Uses the REV Expansion Hub's
     * built in IMU to turn.
     *
     * @param TARGET_ANGLE desired angle to turn to
     * @param power        desired power to turn at
     */
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

    /**
     * Turn left and stop at a desired inputted angle. Uses the REV Expansion Hub's
     * built in IMU to turn. Unlike the normal turnLeft function, this function only
     * moves one side of the robot to turn to create a pivot turn.
     *
     * @param TARGET_ANGLE desired angle to turn to
     * @param power        desired power to turn at
     */
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

    /**
     * Turn right and stop at a desired inputted angle. Uses the REV Expansion Hub's
     * built in IMU to turn. Unlike the normal turnRight function, this function only
     * moves one side of the robot to turn to create a pivot turn.
     *
     * @param TARGET_ANGLE desired angle to turn to
     * @param power        desired power to turn at
     */
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

    /**
     * Turn left and stop at a desired inputted angle. Uses the REV Expansion Hub's
     * built in IMU to turn.
     *
     * @param TARGET_ANGLE desired angle to turn to
     * @param power        desired power to turn at
     */
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

    /**
     * Drive based on an inputted distance and speed. Uses the REV IMU
     * to attempt to drive straight (stay on its current heading, similar to
     * a PID Drive).
     *
     * @param speed    desired speed to move at
     * @param distance desired distance to move
     * @param angle    angle to stay at
     */
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

    /**
     * Strafe using the REV Expansion Hub's built in IMU to
     * stay on a desired heading throughout.
     *
     * @param speed    desired speed to move at
     * @param distance desired distance to move (-distance is right, + is left)
     * @param angle    desired angle to reach/stay on
     */
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

            Mat matLeft = workingMatrix.submat(650, 750, 550, 710);
            Mat matCenter = workingMatrix.submat(650, 750, 910, 1070);
            Mat matRight = workingMatrix.submat(650, 750, 1270, 1430);

            Imgproc.rectangle(workingMatrix, new Rect(550, 650, 160, 100), new Scalar(0, 0, 255));
            Imgproc.rectangle(workingMatrix, new Rect(910, 650, 160, 100), new Scalar(0, 0, 255));
            Imgproc.rectangle(workingMatrix, new Rect(1270, 650, 160, 100), new Scalar(0, 0, 255));

            double leftSum = Core.sumElems(matLeft).val[2];
            double centerSum = Core.sumElems(matCenter).val[2];
            double rightSum = Core.sumElems(matRight).val[2];

            if (leftSum > centerSum) {
                if (leftSum > rightSum) {
                    //skystone is left
                    skystone = Skystone.LEFT;
                }
                else {
                    //skystone is right
                    skystone = Skystone.RIGHT;
                }
            }
            else {
                if (centerSum > rightSum) {
                    //skystone is center
                    skystone = Skystone.MIDDLE;
                }
                else {
                    //skystone is right
                    skystone = Skystone.RIGHT;
                }
            }

            return workingMatrix;
        }
    }
}
