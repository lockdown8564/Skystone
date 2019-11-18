package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@Autonomous(name = "red test", group = "test")
public class SonicRedAutoTest extends LinearOpMode {
    private SonicHardware robot = new SonicHardware();
    @Override
    public void runOpMode(){
        robot.init(hardwareMap);
        robot.hook.setPosition(0.03);

        waitForStart();

        //releaseIntake();
        sleep(800);
        encoderDrive(0.7,-20,-20);
        turnRight(78,0.3);

        robot.stopMotors();
        sleep(300);

        encoderDrive(0.8,47,47);
        turnLeft(25,0.3);

        encoderDriveIntake(1,-20,-20, 1);
        robot.intakeSetPower(1);
        sleep(1000);
        robot.stopMotors();

        encoderDrive(1, 20, 20);
        robot.stopMotors();
        sleep(500);

        turnLeft(75, 0.4);
        encoderDrive(1,50,50);

        turnLeft(168,0.4);
        robot.intakeSetPower(-1);
        sleep(500);
        /*encoderDrive(1,20, 20);
        robot.foundation.setPosition(0.75);
        sleep(500);
        encoderDrive(1,-30, -30);
        turnRight(150,0.1);*/

        turnRight(78,0.3);
        encoderDrive(0.6,20,20);

    }

    private void encoderDrive(double speed, double leftInches, double rightInches){
        int LEFT_TARGET, RIGHT_TARGET;
        if(opModeIsActive()){
            LEFT_TARGET = (int)(leftInches*robot.COUNTS_PER_INCH) + robot.frontLeft.getCurrentPosition();
            RIGHT_TARGET = (int)(rightInches*robot.COUNTS_PER_INCH) + robot.frontRight.getCurrentPosition();

            robot.driveSetMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.driveSetTarget(LEFT_TARGET,RIGHT_TARGET);
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

            robot.driveSetMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.driveSetTarget(LEFT_TARGET,RIGHT_TARGET);
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

    /*
    private void releaseIntake(){
        int TARGET = robot.arm.getCurrentPosition() + 300;
        if(opModeIsActive()){
            robot.arm.setTargetPosition(TARGET);
            robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.arm.setPower(0.5);
            while(robot.arm.isBusy() && opModeIsActive()){
            }

            robot.stopMotors();
            robot.latch.setPosition(0);

            robot.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

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

}
