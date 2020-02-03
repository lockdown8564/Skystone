package testing;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.Arrays;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import hallib.HalDashboard;
import hallib.HalUtil;

public class ShadowTestHardware implements PIDControlTest.PidInput{
    DcMotor frontLeft, frontRight, backLeft, backRight = null;
    private DcMotor lIntake, rIntake = null;
    DcMotor slide = null;
    DcMotor swing = null;
    Servo found1, found2 = null;
    Servo grip = null;

    BNO055IMU imu;
    //private LinearOpMode opMode;

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
    public PIDControlTest pidControl, pidControlTurn;

    static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = 0.15;     // Larger is more responsive, but also less stable

    private final static double SCALE = (WHEEL_DIAMETER_INCHES * Math.PI)/
            (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION); //inches per count
    private double degrees = 0.0;
    private double stallStartTime = 0.0;
    private double prevTime = 0.0;
    private int prevLeftPos = 0;
    private int prevRightPos = 0;
    private boolean slowSpeed;
    private double minTarget, maxTarget;
    //private HalDashboard dashboard;
    private ElapsedTime mRunTime;
    private boolean slow = true;


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

        mRunTime = new ElapsedTime();
        mRunTime.reset();

        //dashboard = ShadowRedFoundAuto.getDashboard();

        pidControl = new PIDControlTest(0.03,0,0,0,0.8,0.2,this);
        pidControlTurn = new PIDControlTest(0.02,0,0,0,0.5,0.2,this);
        pidControlTurn.setAbsoluteSetPoint(true);

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

    //- is right, + is left
    void strafe(int direction, double power){
        driveSetPower(power * direction, -power * direction, -power * direction, power * direction);
    }

    /**
     * void: drivePID(dist, "slow"(T/F), AbortTrigger (object) )
     *
     * This is a command block that gets called when driving.
     *
     * PID implemented -> see PIDControl for details of how PID works.
     *
     * Assuming this is running in a loop
     */
    //Tip: Input distance in inches and power with decimal to hundredth place
    public void drivePID(double distance, boolean slow) {//throws InterruptedException {
        this.degrees = 0;
        resetIntZ();
        //Slow Mode check-----------------------------------------------------------------------
        if(slow) {
            pidControl.setOutputRange(-0.5, 0.5);
        }
        else {
            pidControl.setOutputRange(-0.8,0.8);
        }

        //Distance Check. Checked every tick.
        if (Math.abs(distance) <= 5) {
            pidControl.setPID(0.081,0,0,0);
        }
        else if(Math.abs(distance) <= 10) {
            pidControl.setPID(0.0485,0,0,0);
        }
        else {
            pidControl.setPID(0.0345,0,.0005,0);
        }
        //-------------------------------------------------------------------------

        pidControl.setTarget(distance);//ref drivePID (dist, T/F slow, AbortTrigger)
        pidControlTurn.setTarget(this.degrees);//WARNING: Must be defined if needed. Not provided.
        stallStartTime = HalUtil.getCurrentTime();//using HalUtil time.

        //While loop -----------------------------------------------------------------------------
        //while: pidControl is not on target or pidControlTurn is not on target, and OpMode is active
        while ((!pidControl.isOnTarget() || !pidControlTurn.isOnTarget()) /*&& opMode.opModeIsActive()*/) {

            //safety: abort when needed. Please uncomment when in actual competition.

            //Variable Definition and Assignment -----------------------------------------------
            int currLeftPos = (backLeft.getCurrentPosition() + frontLeft.getCurrentPosition())/2;//this is an encoder output
            int currRightPos = (backRight.getCurrentPosition() + frontRight.getCurrentPosition())/2;//another encoder output
            //-----------------------------------------------------------------------------------
            double drivePower = pidControl.getPowerOutput();//technically a curve, treat as variable (0-1)
            double turnPower = pidControlTurn.getPowerOutput();//same here (-1 - 1) (1 means left)(-1 means right)
            //----------------------------------------------------------------------------

            backLeft.setPower(drivePower + turnPower);
            frontLeft.setPower(drivePower + turnPower);
            backRight.setPower(drivePower - turnPower);
            frontRight.setPower(drivePower - turnPower);

            //--------------------------------------------------------------------------------
            double currTime = HalUtil.getCurrentTime();
            if (currLeftPos != prevLeftPos || currRightPos != prevRightPos) {
                stallStartTime = currTime;
                prevLeftPos = currLeftPos;
                prevRightPos = currRightPos;
            }
            else if (currTime > stallStartTime + 0.15) {
                // The motors are stalled for more than 0.15 seconds.
                break;
            }
            //---------------------------------------------------------------------------
            pidControlTurn.displayPidInfo(0);
            pidControl.displayPidInfo(5);
            //opMode.idle();
        }
        //------------------------------------------------------------------------------------
        //When it is done, or aborted, or anything, it will auto reset motors to 0
        stopMotors();
        resetPIDDrive();
    }

    /**
     * Void: spinPID (deg)
     * Uses PIDControlTurn and the IMU module (gyro sensor)
     *
     * Using orientation (first angle, second angle, third angle
     *
     * Spin the robot to a certain degree, instead of making it going forward or curve
     *
     * Gyro: first angle is Z (heading) second angle is Y (pitch) third angle is X (roll)
     * Imagining flying an airplane, heading is if ur going N/S, Pitch is if ur going up/down,
     * Roll would be how tilted you are.
     *
     * */
    public void spinPID(double degrees) throws InterruptedException {
        resetIntZ();
        //calling in the angle measurements from the gyro
        //usage: angles.firstAngle......etc
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        //resetIntZ();

        //lowering output range for 10 degs angles. so that it turns more accurately.
        if(Math.abs(degrees) < 10.0) {
            pidControlTurn.setOutputRange(-0.45,0.45);
        }
        else {
            pidControlTurn.setOutputRange(-0.75,0.75);
        }

        //------------------------------------------------------------------------------------
        //calling in the degrees for spinning
        this.degrees = degrees;

        if(Math.abs(degrees - intZ()) < 10.0) {  //<10 deg PID dial
            //pidControlTurn.setPID(0.05,0,0.0005,0);
            pidControlTurn.setPID(0.15,0,0.0005,0);
        }
        else if(Math.abs(degrees - intZ()) < 20.0) { //<20deg PID dial
            //pidControlTurn.setPID(0.03,0,0.002,0);
            pidControlTurn.setPID(0.13,0,0.002,0);
        }
        else if(Math.abs(degrees - intZ()) < 45.0) { //<40deg PID dial
            //pidControlTurn.setPID(0.022,0,0.0011,0);
            pidControlTurn.setPID(0.122,0,0.0011,0);
        }
        else if(Math.abs(degrees - intZ()) < 90.0) { //<90deg PID dial
            pidControlTurn.setPID(0.123,0,0.0005,0);
            //pidControlTurn.setPID(0.023,0,0.0005,0);
        }
        else {                                       //More than 90deg PID dial
            pidControlTurn.setPID(0.123,0,0,0);
            //pidControlTurn.setPID(0.023,0,0,0)
        }

        //-----------------------------------------------------------------------------------
        pidControlTurn.setTarget(degrees);//sets target degrees.
        stallStartTime = HalUtil.getCurrentTime();//check time

        //----------------------------------------------------------------------------------
        //While: pidControlTurn not on target and OpMode is Active

        while (!pidControlTurn.isOnTarget() /*&& opMode.opModeIsActive()*/) {

            //-----------------------------------------------------------------------------
            //Encoder Position Check
            int currLeftPos = (backLeft.getCurrentPosition() + frontLeft.getCurrentPosition())/2;
            int currRightPos = (backRight.getCurrentPosition() + frontRight.getCurrentPosition())/2;

            //Output Check generated from PID (results)
            double outputPower = pidControlTurn.getPowerOutput();

            //set Power
            backLeft.setPower(outputPower);
            frontLeft.setPower(outputPower);
            backRight.setPower(-outputPower);
            frontRight.setPower(-outputPower);

            //------------------------------------------------------------------------------

            //Stall Check--------------------------------------------------------------------
            double currTime = HalUtil.getCurrentTime();//time check
            if (currLeftPos != prevLeftPos || currRightPos != prevRightPos)
            {
                stallStartTime = currTime;
                prevLeftPos = currLeftPos;
                prevRightPos = currRightPos;
            }
            else if (currTime > stallStartTime + 1)
            {
                // The motors are stalled for more than 1 seconds.
                break;
            }
            //---------------------------------------------------------------------------------

            pidControlTurn.displayPidInfo(0);
            //opMode.idle();
        }

        //sets it to 0 when done.
        stopMotors();
        resetPIDDrive();//reset
        resetIntZ();
    }

    /**
     * This method drives the motors at "magnitude" and "curve". Both magnitude and curve are -1.0 to +1.0 values,
     * where 0.0 represents stopped and not turning. curve < 0 will turn left and curve > 0 will turn right. The
     * algorithm for steering provides a constant turn radius for any normal speed range, both forward and backward.
     * Increasing sensitivity causes sharper turns for fixed values of curve.
     *
     * @param magnitude specifies the speed setting for the outside wheel in a turn, forward or backwards, +1 to -1.
     * @param curve specifies the rate of turn, constant for different forward speeds. Set curve < 0 for left turn or
     *              curve > 0 for right turn. Set curve = e^(-r/w) to get a turn radius r for wheelbase w of your
     *              robot. Conversely, turn radius r = -ln(curve)*w for a given value of curve and wheelbase w.
     * @param inverted specifies true to invert control (i.e. robot front becomes robot back).
     */
    public void curve(double magnitude, double curve, boolean inverted, boolean gyroAssist)
    {
        double leftOutput;
        double rightOutput;
        double sensitivity = 0.5;

        if (curve < 0.0)
        {
            double value = Math.log(-curve);
            double ratio = (value - sensitivity)/(value + sensitivity);
            if (ratio == 0.0)
            {
                ratio = 0.0000000001;
            }
            leftOutput = magnitude/ratio;
            rightOutput = magnitude;
        }
        else if (curve > 0.0)
        {
            double value = Math.log(curve);
            double ratio = (value - sensitivity)/(value + sensitivity);
            if (ratio == 0.0)
            {
                ratio = 0.0000000001;
            }
            leftOutput = magnitude;
            rightOutput = magnitude/ratio;
        }
        else
        {
            leftOutput = magnitude;
            rightOutput = magnitude;
        }

        curveDrive(leftOutput, rightOutput, inverted, gyroAssist);
    }

    /**
     * This method implements tank drive where leftPower controls the left motors and right power controls the right
     * motors.
     *
     * @param leftPower specifies left power value.
     * @param rightPower specifies right power value.
     * @param inverted specifies true to invert control (i.e. robot front becomes robot back).
     */
    public void curveDrive(double leftPower, double rightPower, boolean inverted, boolean gyroAssist)
    {

        double currTime = HalUtil.getCurrentTime();

        leftPower = HalUtil.clipRange(leftPower);
        rightPower = HalUtil.clipRange(rightPower);

        double gyroRateScale = 0.0;
        double gyroAssistKp = 1.0;

        if (inverted)
        {
            double swap = leftPower;
            leftPower = -rightPower;
            rightPower = -swap;
        }

        if(gyroAssist)
        {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double diffPower = (leftPower - rightPower)/2.0;
            double assistPower = HalUtil.clipRange(gyroAssistKp*(diffPower - gyroRateScale*(intZ()/(currTime-prevTime))));
            leftPower += assistPower;
            rightPower -= assistPower;
            double maxMag = Math.max(Math.abs(leftPower), Math.abs(rightPower));
            if (maxMag > 1.0)
            {
                leftPower /= maxMag;
                rightPower /= maxMag;
            }
        }

        leftPower = HalUtil.clipRange(leftPower, -1.0, 1.0);
        rightPower = HalUtil.clipRange(rightPower, -1.0, 1.0);

        frontLeft.setPower(leftPower);
        backLeft.setPower(leftPower);
        frontRight.setPower(rightPower);
        backRight.setPower(rightPower);

        prevTime = currTime;

    }

    /**
     * slowSpeed sets if the robot should go slowly
     *
     * @param slow is whether or not the robot should go slow
     */
    public void slowSpeed(boolean slow) {
        slowSpeed = slow;
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

    void driveSetTargetInd(int flTarget, int frTarget, int blTarget, int brTarget){
        frontLeft.setTargetPosition(flTarget);
        backLeft.setTargetPosition(blTarget);
        frontRight.setTargetPosition(frTarget);
        backRight.setTargetPosition(brTarget);
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

    public void resetPIDDrive() {
        pidControl.reset();
        pidControlTurn.reset();
    }

    @Override
    public double getInput(PIDControlTest pidCtrl) {
        double input = 0.0;
        if (pidCtrl == pidControl) {
            input = (backLeft.getCurrentPosition() + frontLeft.getCurrentPosition() + frontRight.getCurrentPosition() + backRight.getCurrentPosition())*SCALE/4.0;
        }
        else if (pidCtrl == pidControlTurn) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            //input = gyro.getHeading();
            input = intZ();
        }

        return input;
    }

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
