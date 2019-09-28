package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

class SonicHardware {
    DcMotor frontLeft, frontRight, backLeft, backRight = null;
    /*BNO055IMU imu;
    RevBlinkinLedDriver ledDriver;
    RevBlinkinLedDriver.BlinkinPattern pattern;
    Orientation angles;*/
    private HardwareMap hwMap;

    void init(HardwareMap ahwmap){
        hwMap = ahwmap;
        frontLeft = hwMap.get(DcMotor.class,"fl");
        frontRight = hwMap.get(DcMotor.class,"fr");
        backLeft = hwMap.get(DcMotor.class,"bl");
        backRight = hwMap.get(DcMotor.class,"br");

        /*imu = hwMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);*/

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        stopMotors();

        /*frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);*/

        //TODO: add some methods to help with teleop and auto
    }
    void driveSetPower(double lPower, double rPower) {
        frontLeft.setPower(lPower);
        frontRight.setPower(rPower);
        backLeft.setPower(lPower);
        backRight.setPower(rPower);
    }

    void driveSetMode(DcMotor.RunMode mode){
        frontLeft.setMode(mode);
        frontRight.setMode(mode);
        backLeft.setMode(mode);
        backRight.setMode(mode);
    }

    void driveSetPowerAll(double power){
        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);
    }

    void stopMotors(){
        driveSetPowerAll(0);
    }
}
