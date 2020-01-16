package testing;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import java.util.Arrays;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class ShadowTestHardware {
    DcMotor frontLeft, frontRight, backLeft, backRight = null;
    private DcMotor lIntake, rIntake = null;
    DcMotor slide = null;
    DcMotor swing = null;
    Servo found1, found2 = null;
    Servo grip = null;

    BNO055IMU imu;
    private static final double     COUNTS_PER_MOTOR_REV    = 1120 ; //neverest 40
    private static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;  //1:1
    private static final double     WHEEL_DIAMETER_INCHES   = 3.93701 ; //10 cm
    private static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION)/
            (WHEEL_DIAMETER_INCHES * Math.PI);
    /*RevBlinkinLedDriver ledDriver;
    RevBlinkinLedDriver.BlinkinPattern pattern;*/
    Orientation angles;
    private HardwareMap hwMap;
    private double flPower, frPower, blPower, brPower = 0;
    DigitalChannel touch = null;
    ColorSensor color = null;

    public enum DriveDirection{
        FORWARD,
        REVERSE
    }

    public void init(HardwareMap ahwmap){
        hwMap = ahwmap;
        frontLeft = hwMap.get(DcMotor.class,"fl");
        frontRight = hwMap.get(DcMotor.class,"fr");
        backLeft = hwMap.get(DcMotor.class,"bl");
        backRight = hwMap.get(DcMotor.class,"br");
        lIntake = hwMap.get(DcMotor.class,"lIn");
        rIntake = hwMap.get(DcMotor.class,"rIn");
        swing = hwMap.get(DcMotor.class,"swing");
        slide = hwMap.get(DcMotor.class,"slide");

        found1 = hwMap.get(Servo.class,"found1");
        found2 = hwMap.get(Servo.class,"found2");
        grip = hwMap.get(Servo.class,"grip");

        touch = hwMap.get(DigitalChannel.class, "touch");
        color = hwMap.get(ColorSensor.class,"color");
        imu = hwMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);

        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        lIntake.setDirection(DcMotorSimple.Direction.FORWARD);
        rIntake.setDirection(DcMotorSimple.Direction.FORWARD);

        swing.setDirection(DcMotorSimple.Direction.FORWARD);
        slide.setDirection(DcMotorSimple.Direction.FORWARD);

        driveSetZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        swing.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        driveSetMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveSetMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        swing.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        swing.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        touch.setMode(DigitalChannel.Mode.INPUT);

        stopMotors();
    }
    public void driveSetPower(double flPower, double frPower, double blPower, double brPower) {
        frontLeft.setPower(flPower);
        frontRight.setPower(frPower);
        backLeft.setPower(blPower);
        backRight.setPower(brPower);
    }

    public void mecanumDrive(double forward, double strafe, double turn, DriveDirection direction, double maxSpeed){
        double num = 1;
        switch(direction){
            case FORWARD:{
                break;
            }
            case REVERSE:{
                num = -1;
                break;
            }
        }

        flPower = forward + strafe + turn;
        frPower = forward - strafe - turn;
        blPower = forward - strafe + turn;
        brPower = forward + strafe - turn;
        setSpeedsMec(flPower * num, frPower * num,
                blPower * num, brPower * num, maxSpeed);
    }

    private void setSpeedsMec(double frontLeftPower, double frontRightPower, double backLeftPower, double backRightPower, double max){
        double[] arr = {1.0, frontLeftPower, frontRightPower, backLeftPower, backRightPower};
        Arrays.sort(arr);
        double largest = arr[4];

        driveSetPower((frontLeftPower/largest) * max, (frontRightPower/largest) * max,
                (backLeftPower/largest) * max, (backRightPower/largest) * max);
    }

    //- is left, + is right
    void strafe(int direction, double power){
        driveSetPower(power * direction, -power * direction, -power * direction, power * direction);
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

    void gripFoundation(){
        found1.setPosition(1);
        found2.setPosition(0);
    }

    void releaseFoundation(){
        found1.setPosition(0.25);
        found2.setPosition(0.95);
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
        lIntake.setPower(0);
        rIntake.setPower(0);
        slide.setPower(0);
        swing.setPower(0);
    }

    void intakeSetPower(double power){
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
