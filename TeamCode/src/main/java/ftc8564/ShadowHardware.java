package ftc8564;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImpl;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Arrays;

import hallib.HalUtil;

public class ShadowHardware {
    DcMotor frontLeft, frontRight, backLeft, backRight = null;
    DcMotor lIntake, rIntake = null;
    DcMotor slide = null;
    DcMotor swing = null;
    Servo found1, found2 = null;
    Servo grip = null;
    CRServo yeet1, yeet2 = null;

    BNO055IMU imu;

    private static final double     COUNTS_PER_MOTOR_REV    = 1120 ; //neverest 40
    private static final double     DRIVE_GEAR_REDUCTION    = 0.66666667 ;  //24:16
    private static final double     WHEEL_DIAMETER_INCHES   = 3.93700787402 ; //10 cm
    private static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION)/
            (WHEEL_DIAMETER_INCHES * Math.PI);

    /*RevBlinkinLedDriver ledDriver;
    RevBlinkinLedDriver.BlinkinPattern pattern;*/

    Orientation angles;
    private double prevAngle = 0;

    private HardwareMap hwMap;
    private double flPower, frPower, blPower, brPower = 0;
    DigitalChannel touch = null;
    ColorSensor color = null;

    static final double     P_TURN_COEFF            = 0.1;
    static final double     P_DRIVE_COEFF           = 0.1;

    private final static double SCALE = (WHEEL_DIAMETER_INCHES * Math.PI)/
            (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION); //inches per count

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
        yeet1 = hwMap.get(CRServo.class,"yeet1");
        yeet2 = hwMap.get(CRServo.class,"yeet2");

        touch = hwMap.get(DigitalChannel.class, "touch");
        color = hwMap.get(ColorSensor.class,"color");
        imu = hwMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu.initialize(parameters);

        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        lIntake.setDirection(DcMotorSimple.Direction.FORWARD);
        rIntake.setDirection(DcMotorSimple.Direction.FORWARD);

        swing.setDirection(DcMotorSimple.Direction.REVERSE);
        slide.setDirection(DcMotorSimple.Direction.FORWARD);

        yeet1.setDirection(CRServoImpl.Direction.FORWARD);
        yeet2.setDirection(CRServoImpl.Direction.FORWARD);

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

        releaseFoundation();
        stopMotors();
    }

    public double intZ(){
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return (prevAngle - angles.firstAngle);
    }
    public void resetIntZ(){
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        prevAngle = angles.firstAngle;
    }

    /**
     * set power of each individual motor
     *
     * @param flPower power to give front left motor
     * @param frPower power to give front right motor
     * @param blPower power to give back left motor
     * @param brPower power to give back right motor
     */
    public void driveSetPower(double flPower, double frPower, double blPower, double brPower) {
        frontLeft.setPower(flPower);
        frontRight.setPower(frPower);
        backLeft.setPower(blPower);
        backRight.setPower(brPower);
    }

    /**
     * sleep stops the program for number of seconds
     *
     * @param seconds is the time in seconds to wait
     */
    public void sleep(double seconds)
    {
        double startTime = HalUtil.getCurrentTime();
        while(startTime + seconds > HalUtil.getCurrentTime()) {}
    }

    /**
     * This method accepts all of the inputs for mecanum drive and does
     * some basic math to output values into a method to move the robot.
     *
     * @param forward   the "y" value of the left joystick; determines the robot's
     *                  forwards and backwards movement
     * @param strafe    the "x" value of the left joystick; determines the robot's
     *                  strafing movement
     * @param turn      the "x" value of the right joystick; controls the robot turning
     * @param direction if the robot is in forward or reverse mode
     * @param maxSpeed  the max speed of driving
     */
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

    /**
     * This robot sets the speed of the motors based on inputs from
     * the joysticks and mecanumDrive method. In addition, the method
     * scales down the powers from the greatest input.
     *
     * @param frontLeftPower  power to give front left motor
     * @param frontRightPower power to give front right motor
     * @param backLeftPower   power to give back left motor
     * @param backRightPower  power to give back right motor
     * @param max             the max speed of driving (slow mode)
     */
    private void setSpeedsMec(double frontLeftPower, double frontRightPower, double backLeftPower,
                              double backRightPower, double max){
        double[] arr = {1.0, frontLeftPower, frontRightPower, backLeftPower, backRightPower};
        Arrays.sort(arr);
        double largest = arr[4];

        driveSetPower((frontLeftPower/largest) * max, (frontRightPower/largest) * max,
                (backLeftPower/largest) * max, (backRightPower/largest) * max);
    }

    /**
     * Strafe left or right based on the inputted direction and power.
     * - is right, + is left
     *
     * @param direction -1 or 1, determines left or right
     * @param power     power to strafe at
     */
    void strafe(int direction, double power){
        driveSetPower(power * direction, -power * direction, -power * direction, power * direction);
    }

    /**
     * Sets the drive mode of all the drive motors.
     *
     * @param mode runmode to set the motors to, usually run to position or
     *             run using encoders
     */
    void driveSetMode(DcMotor.RunMode mode){
        frontLeft.setMode(mode);
        frontRight.setMode(mode);
        backLeft.setMode(mode);
        backRight.setMode(mode);
    }

    /**
     * Sets the zero power behavior of all the drive motors
     *
     * @param zero zero power behavior to set the motors to, brake or float
     */
    void driveSetZeroPowerBehavior(DcMotor.ZeroPowerBehavior zero){
        frontLeft.setZeroPowerBehavior(zero);
        frontRight.setZeroPowerBehavior(zero);
        backLeft.setZeroPowerBehavior(zero);
        backRight.setZeroPowerBehavior(zero);
    }

    /**
     * bring both servos to their downwards position to grip the foundation
     */
    void releaseFoundation(){
        found1.setPosition(0.9);
        found2.setPosition(0.1);
    }

    /**
     * bring both servos to their upwards position to release the foundation
     */
    void gripFoundation(){
        found1.setPosition(0);
        found2.setPosition(1);
    }

    /**
     * Set the targets of the left and right motors.
     *
     * @param lTarget target of left motors
     * @param rTarget target of right motors
     */
    void driveSetTarget(int lTarget, int rTarget){
        frontLeft.setTargetPosition(lTarget);
        backLeft.setTargetPosition(lTarget);
        frontRight.setTargetPosition(rTarget);
        backRight.setTargetPosition(rTarget);
    }

    /**
     * Set the targets of each individual motor.
     *
     * @param flTarget target of front left motor
     * @param frTarget target of front right motor
     * @param blTarget target of back left motor
     * @param brTarget target of back right motor
     */
    void driveSetTargetInd(int flTarget, int frTarget, int blTarget, int brTarget){
        frontLeft.setTargetPosition(flTarget);
        backLeft.setTargetPosition(blTarget);
        frontRight.setTargetPosition(frTarget);
        backRight.setTargetPosition(brTarget);
    }

    /**
     * Set the power of all motors to the same speed
     *
     * @param power the speed to set the motors to
     */
    void driveSetPowerAll(double power){
        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);
    }

    /**
     * Stop all movement of motors.
     */
    public void stopMotors(){
        driveSetPowerAll(0);
        lIntake.setPower(0);
        rIntake.setPower(0);
        slide.setPower(0);
        swing.setPower(0);
    }

    /**
     * Runs the intake at a desired speed.
     * - is in + is out
     *
     * @param power the power to run the intake motors at
     */
    void intakeSetPower(double power){
        lIntake.setPower(power);
        rIntake.setPower(power);
    }

    /**
     * Checks if the motors are busy (used for encoders)
     *
     * @return if all of the motors are busy
     */
    boolean driveIsBusy(){
        return frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy();
    }

    /**
     * Returns counts per inch (for encoders)
     *
     * @return counts per inch variable
     */
    double getCPI(){
        return COUNTS_PER_INCH;
    }

    /**
     * returns how off the robot is from the desired heading
     * if it is less than or greater than 180, add or subtract
     * 360 to get the true value the robot is off
     *
     * @param targetAngle the desired heading to stay on
     * @return how far off the robot is from desired heading
     */
    public double getError(double targetAngle) {
        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - intZ();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return steer
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }
}
