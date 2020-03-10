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
 * This autonomous program was intended to score the foundation then park.
 *
 * @author William Trang
 * @version 3.0 2/2/20
 */

@Autonomous(name = "blue found park", group = "test")
public class ShadowBlueFoundAuto extends LinearOpMode{
    private ShadowTestHardware robot = new ShadowTestHardware();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        waitForStart();

        //move forward and grip foundation
        encoderDrive(0.5,-35,-35);
        robot.gripFoundation();
        sleep(1000);

        //turn and place foundation
        turnLeftCurvy(85,0.15);
        encoderDrive(0.7,-30,-30);
        robot.releaseFoundation();
        sleep(500);
        encStrafe(0.5, 9);
        encoderDrive(0.7,55,55);
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
            while(currentAngle<=TARGET_ANGLE){
                currentAngle = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                robot.driveSetPower(power*6, power, power*6, power);
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

}
