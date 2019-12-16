package rip_sonic;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class SonicHardware {
    DcMotor frontLeft, frontRight, backLeft, backRight = null;
    private DcMotor lIntake, rIntake = null;
    DcMotor slide = null;
    DcMotor winch = null;
    public Servo latch, hook, arm = null;
    DigitalChannel touch = null;
    BNO055IMU imu;
    private static final double     COUNTS_PER_MOTOR_REV    = 723.24 ; //5201 Spur Gear 26:1
    private static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;  //1:1
    private static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;
    private static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION)/
            (WHEEL_DIAMETER_INCHES * Math.PI);
    /*RevBlinkinLedDriver ledDriver;
    RevBlinkinLedDriver.BlinkinPattern pattern;*/
    Orientation angles;
    private HardwareMap hwMap;

    public void init(HardwareMap ahwmap){
        hwMap = ahwmap;
        frontLeft = hwMap.get(DcMotor.class,"fl");
        frontRight = hwMap.get(DcMotor.class,"fr");
        backLeft = hwMap.get(DcMotor.class,"bl");
        backRight = hwMap.get(DcMotor.class,"br");
        lIntake = hwMap.get(DcMotor.class,"lIn");
        rIntake = hwMap.get(DcMotor.class,"rIn");
        slide = hwMap.get(DcMotor.class,"slide");
        winch = hwMap.get(DcMotor.class,"winch");

        arm = hwMap.get(Servo.class,"arm");
        hook = hwMap.get(Servo.class,"hook");
        latch = hwMap.get(Servo.class,"latch");

        imu = hwMap.get(BNO055IMU.class, "imu");
        touch = hwMap.get(DigitalChannel.class,"touch");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        lIntake.setDirection(DcMotorSimple.Direction.FORWARD);
        rIntake.setDirection(DcMotorSimple.Direction.REVERSE);

        slide.setDirection(DcMotorSimple.Direction.FORWARD);
        winch.setDirection(DcMotorSimple.Direction.FORWARD);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        winch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        winch.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        stopMotors();

        //TODO: add some methods to help with teleop and auto
    }
    public void driveSetPower(double lPower, double rPower) {
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

    void driveSetZeroPowerBehavior(DcMotor.ZeroPowerBehavior zero){
        frontLeft.setZeroPowerBehavior(zero);
        frontRight.setZeroPowerBehavior(zero);
        backLeft.setZeroPowerBehavior(zero);
        backRight.setZeroPowerBehavior(zero);
    }

    void driveSetTarget(int lTarget, int rTarget){
        frontLeft.setTargetPosition(lTarget);
        backLeft.setTargetPosition(lTarget);
        frontRight.setTargetPosition(rTarget);
        backRight.setTargetPosition(rTarget);
    }

    void driveSetPowerAll(double power){
        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);
    }

    public void stopMotors(){
        driveSetPowerAll(0);
        slide.setPower(0);
        lIntake.setPower(0);
        rIntake.setPower(0);
        winch.setPower(0);
    }

    public void intakeSetPower(double power){
        lIntake.setPower(power);
        rIntake.setPower(power);
    }

    boolean driveIsBusy(){
        return frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy();
    }

    double getCPI(){
        return COUNTS_PER_INCH;
    }

}
