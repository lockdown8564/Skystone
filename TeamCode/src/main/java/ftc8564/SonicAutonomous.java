package ftc8564;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import org.firstinspires.ftc.robotcore.external.ClassFactory;

import ftclib.FtcChoiceMenu;
import ftclib.FtcMenu;
import ftclib.FtcValueMenu;
import hallib.HalDashboard;

/**
 * first auto on red side
 * made 11/1/19
 * last updated: 11/1/19
 */
@Disabled
@Autonomous(name = "Sonic Autonomous", group = "ftc8564")
public class SonicAutonomous extends LinearOpMode implements FtcMenu.MenuButtons{
    private SonicHardware robot = new SonicHardware();
    private enum Alliance{
        BLUE,
        RED
    }
    private enum StartingSide{
        FOUNDATION,
        STONES
    }
    private enum Skystones{
        ZERO,
        ONE,
        TWO
    }
    private enum Foundation{
        YES,
        NO
    }
    private enum ParkingSide{
        CENTER,
        WALL
    }
    @Override
    public void runOpMode(){
        robot.init(hardwareMap);
        robot.driveSetMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        waitForStart();

        releaseIntake();
        encoderDrive(0.7,-24,-24);
    }

    private void encoderDrive(double speed, double leftInches, double rightInches){
        int LEFT_TARGET, RIGHT_TARGET;
        if(opModeIsActive()){
            LEFT_TARGET = (int)(leftInches*robot.COUNTS_PER_INCH) + robot.frontLeft.getCurrentPosition();
            RIGHT_TARGET = (int)(rightInches*robot.COUNTS_PER_INCH) + robot.frontRight.getCurrentPosition();

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
            LEFT_TARGET = (int)(leftInches*robot.COUNTS_PER_INCH) + robot.frontLeft.getCurrentPosition();
            RIGHT_TARGET = (int)(rightInches*robot.COUNTS_PER_INCH) + robot.frontRight.getCurrentPosition();

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

    private void turnRight(final float TARGET_ANGLE, double power){
        while(opModeIsActive()){
            float currentAngle = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            if(currentAngle>=-TARGET_ANGLE){
                robot.driveSetPower(-power,power);
            }
            else{
                robot.stopMotors();
                break;
            }
        }
    }

    private void turnLeft(final float TARGET_ANGLE, double power){
        while(opModeIsActive()){
            float currentAngle = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            if(currentAngle<=TARGET_ANGLE){
                robot.driveSetPower(power,-power);
            }
            else{
                robot.stopMotors();
                break;
            }
        }
    }


    private void releaseIntake(){
        robot.hook.setPosition(0.9);
        robot.latch.setPosition(0);
    }
    /*
    private void resetArm(int TARGET){
        if(opModeIsActive()){
            robot.arm.setTargetPosition(TARGET);
            robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.arm.setPower(0.5);
            while(robot.arm.isBusy() && opModeIsActive()){
            }

            robot.stopMotors();
            robot.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }*/

    @Override
    public boolean isMenuUpButton() { return gamepad1.dpad_up; }

    @Override
    public boolean isMenuDownButton() { return gamepad1.dpad_down; }

    @Override
    public boolean isMenuEnterButton() { return gamepad1.dpad_right; }

    @Override
    public boolean isMenuBackButton() { return gamepad1.dpad_left; }

}
