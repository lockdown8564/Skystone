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
 * Red stone autonomous test for position right next to the wall. Uses OpenCV
 * to read the orientation of the stones from initial position. From there, grabs the
 * skystone and parks under the bridge. This program relies heavily on encoders
 * for both strafing and moving forward and back.
 *
 * @author William Trang
 * @version 2.1 2/16/20
 */

@Autonomous(name = "red ss wall", group = "test")
public class ShadowRedSSWall extends LinearOpMode{
    private ShadowTestHardware robot = new ShadowTestHardware();

    //create opmode members
    private OpenCvCamera webcam;
    private SSDetector detector;
    private Skystone skystone = Skystone.RIGHT;

    private enum Skystone{
        /**
         * left position of skystone
         */
        LEFT,

        /**
         * center position of skystone
         */
        MIDDLE,

        /**
         * right position of skystone
         */
        RIGHT
    }

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        //create and open camera device
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        detector = new SSDetector();
        webcam.openCameraDevice();
        webcam.setPipeline(detector);
        webcam.startStreaming(1920, 1080, OpenCvCameraRotation.UPRIGHT);

        //update position on phone before match starts
        while(!isStarted()) {
            skystone = detector.skystone;
            telemetry.addData("skystone:", skystone);
            telemetry.update();
        }

        waitForStart();

        switch(skystone){
            case LEFT:{
                //drive forward to stones
                encoderDrive(0.7, -24, -24);
                encStrafe(0.5, -6);
                robot.stopMotors();

                //turn toward left stone and strafe diagonal
                turnRight(120, 0.4);
                encStrafeFlBr(0.6, 15);
                encoderDriveIntake(0.7, 16, 16, -1);

                robot.intakeSetPower(-1);
                sleep(1000);
                robot.stopMotors();

                encStrafeFlBrIntake(0.7, -50, -1);
                turnLeft(84, -0.4);
                turnRight(-84, 0.4);

                encoderDrive(0.7, 100, 100);
                sleep(500);
                robot.stopMotors();
                encoderDrive(0.7, -8, -8);

                break;
            }

            case MIDDLE:{
                encoderDrive(0.7,-10,-10);
                encStrafe(0.5, -6);
                robot.stopMotors();

                turnRight(175, 0.5);
                encImuDriveIntake(0.45, 57, robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle, -1);

                robot.intakeSetPower(-1);
                sleep(1000);
                robot.stopMotors();

                encoderDrive(0.7, -66, -66);
                encoderDrive(0.7, 4, 4);
                turnLeft(84, -0.4);
                turnRight(-90, 0.4);

                encoderDrive(0.7, 75, 75);
                sleep(500);
                robot.stopMotors();
                encoderDrive(0.7, -13, -13);

                break;
            }

            case RIGHT:{
                encoderDrive(0.7,-10,-10);
                encStrafe(0.5, -13);
                robot.stopMotors();

                turnRight(175, 0.5);
                encImuDriveIntake(0.45, 66, robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle, -1);

                robot.intakeSetPower(-1);
                sleep(1000);
                robot.stopMotors();

                encoderDrive(0.7, -72, -72);
                encoderDrive(0.7, 4, 4);
                turnLeft(84, -0.4);
                turnRight(-90, 0.4);

                encoderDrive(0.7, 70, 70);
                sleep(500);
                robot.stopMotors();
                encoderDrive(0.7, -12, -12);

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
            float currentAngle = robot.imu.getAngularOrientation(AxesReference.INTRINSIC,
                    AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            while(currentAngle>=-TARGET_ANGLE){
                currentAngle = robot.imu.getAngularOrientation(AxesReference.INTRINSIC,
                        AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
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
            float currentAngle = robot.imu.getAngularOrientation(AxesReference.INTRINSIC,
                    AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            while(currentAngle<=TARGET_ANGLE){
                currentAngle = robot.imu.getAngularOrientation(AxesReference.INTRINSIC,
                        AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
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

    private void encImuDriveIntake (double speed, double distance, double angle, double direction){
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

                robot.intakeSetPower(direction);
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

    /**
     * Strafe specific distances based on input.
     * - is strafe left, + is strafe right
     * @param speed    speed to move at
     * @param distance distance to move
     */
    private void encStrafe(double speed, double distance){
        int flTarget, frTarget, blTarget, brTarget;

        if(opModeIsActive()){
            flTarget = (robot.frontLeft.getCurrentPosition()) + (int)(distance*robot.getCPI());
            frTarget = (robot.frontRight.getCurrentPosition()) - (int)(distance*robot.getCPI());
            blTarget = (robot.backLeft.getCurrentPosition()) - (int)(distance*robot.getCPI());
            brTarget = (robot.backRight.getCurrentPosition()) + (int)(distance*robot.getCPI());

            robot.driveSetTargetInd(flTarget, frTarget, blTarget, brTarget);
            robot.driveSetMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.driveSetPowerAll(Math.abs(speed));

            while(robot.driveIsBusy() && opModeIsActive()){
            }

            robot.stopMotors();
            robot.driveSetMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     * Strafe diagonally using only the front left and back right motors
     *
     * @param speed    speed to move at
     * @param distance distance to move
     */
    private void encStrafeFlBr(double speed, double distance){
        int flTarget, brTarget;

        if(opModeIsActive()){
            flTarget = (robot.frontLeft.getCurrentPosition()) + (int)(distance*robot.getCPI());
            brTarget = (robot.backRight.getCurrentPosition()) + (int)(distance*robot.getCPI());

            robot.frontLeft.setTargetPosition(flTarget);
            robot.backRight.setTargetPosition(brTarget);

            robot.driveSetMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.driveSetPower(Math.abs(speed), 0, 0 ,Math.abs(speed));

            while(robot.driveIsBusy() && opModeIsActive()){
            }

            robot.stopMotors();
            robot.driveSetMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     * Strafe diagonally while intaking using only the front left and back right motors
     *
     * @param speed     speed to move at
     * @param distance  distance to move at
     * @param direction direction to intake (- for in + for out)
     */
    private void encStrafeFlBrIntake(double speed, double distance, double direction){
        int flTarget, brTarget;

        if(opModeIsActive()){
            flTarget = (robot.frontLeft.getCurrentPosition()) + (int)(distance*robot.getCPI());
            brTarget = (robot.backRight.getCurrentPosition()) + (int)(distance*robot.getCPI());

            robot.frontLeft.setTargetPosition(flTarget);
            robot.backRight.setTargetPosition(brTarget);

            robot.driveSetMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.driveSetPower(Math.abs(speed), 0, 0 ,Math.abs(speed));

            while(robot.driveIsBusy() && opModeIsActive()){
                robot.intakeSetPower(direction);
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

            Mat matLeft = workingMatrix.submat(710, 810, 600, 760);
            Mat matCenter = workingMatrix.submat(710, 810, 960, 1120);
            Mat matRight = workingMatrix.submat(710, 810, 1320, 1480);

            Imgproc.rectangle(workingMatrix, new Rect(600, 710, 160, 100), new Scalar(0, 0, 255));
            Imgproc.rectangle(workingMatrix, new Rect(960, 710, 160, 100), new Scalar(0, 0, 255));
            Imgproc.rectangle(workingMatrix, new Rect(1320, 710, 160, 100), new Scalar(0, 0, 255));

            double leftSum = Core.sumElems(matLeft).val[2];
            double centerSum = Core.sumElems(matCenter).val[2];
            double rightSum = Core.sumElems(matRight).val[2];

            if (leftSum > centerSum) {
                if (leftSum > rightSum) {
                    skystone = Skystone.LEFT;
                } else {
                    skystone = Skystone.RIGHT;
                }
            } else {
                if (centerSum > rightSum) {
                    skystone = Skystone.MIDDLE;
                } else {
                    skystone = Skystone.RIGHT;
                }
            }
            return workingMatrix;
        }
    }
}
