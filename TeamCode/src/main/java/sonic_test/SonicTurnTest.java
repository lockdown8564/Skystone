package sonic_test;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;

/**
 * program to test imu functionality
 * created: 12/14/19
 * last updated: 12/14/19
 */
@Disabled
@TeleOp(name = "imu test", group = "test")
public class SonicTurnTest extends LinearOpMode {
    BNO055IMU imu;

    private SonicTestHardware robot = new SonicTestHardware();
    private DriveMode driveMode = DriveMode.TANK;
    private DriveSpeed driveSpeed = DriveSpeed.FAST;
    private DriveDirection driveDirection = DriveDirection.FORWARD;

    private double lPower, rPower, drivePower, turnPower = 0;
    private double num = 1;
    private int initialSlide = 0;

    private enum DriveMode{
        ARCADE,
        TANK,
    }
    private enum DriveSpeed{
        FAST,
        SLOW
    }
    private enum DriveDirection{
        FORWARD,
        REVERSE
    }

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    //----------------------------------------------------------------------------------------------
    // Main logic
    //----------------------------------------------------------------------------------------------

    @Override public void runOpMode() {

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Set up our telemetry dashboard
        composeTelemetry();

        // Wait until we're told to go
        waitForStart();

        // Start the logging of measured acceleration
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        // Loop and update the dashboard
        while (opModeIsActive()) {
            telemetry.update();
        }

        if(gamepad1.a && driveMode == DriveMode.TANK){
            driveMode = DriveMode.ARCADE;
        }

        else if(gamepad1.b && driveMode == DriveMode.ARCADE){
            driveMode = DriveMode.TANK;
        }

        if(gamepad1.right_trigger != 0){
            driveSpeed = DriveSpeed.SLOW;
            num = 0.5;
        }

        else {
            driveSpeed = DriveSpeed.FAST;
            num = 1;
        }

        if(gamepad1.y){
            driveDirection = DriveDirection.FORWARD;
        }

        else if(gamepad1.x){
            driveDirection = DriveDirection.REVERSE;
        }

        //TODO: clean up with methods
        switch(driveMode){
            case TANK:{
                switch(driveDirection) {
                    case REVERSE:
                        lPower = -gamepad1.left_stick_y;
                        rPower = -gamepad1.right_stick_y;
                        robot.driveSetPower(lPower * num, rPower * num);
                        break;
                    case FORWARD:
                        lPower = gamepad1.right_stick_y;
                        rPower = gamepad1.left_stick_y;
                        robot.driveSetPower(lPower * num, rPower * num);
                        break;
                }
                break;
            }
            case ARCADE:{
                switch(driveDirection) {
                    case REVERSE:
                        drivePower = -gamepad1.left_stick_y;
                        turnPower = gamepad1.right_stick_x;
                        robot.driveSetPower((drivePower+turnPower)*num,(drivePower-turnPower)*num);
                        break;
                    case FORWARD:
                        drivePower = gamepad1.left_stick_y;
                        turnPower = gamepad1.right_stick_x;
                        robot.driveSetPower((drivePower + turnPower)*num,(drivePower - turnPower)*num);
                        break;
                }
                break;
            }
        }

        //down = in
        if(gamepad2.right_stick_y > 0) {
            robot.intakeSetPower(-1);
        }
        //up = out
        else if(gamepad2.right_stick_y < 0){
            robot.intakeSetPower(1);
        }

        else{
            robot.intakeSetPower(0);
        }

        if(gamepad2.a){
            robot.hook.setPosition(1);
        }

        else if(gamepad2.b){
            robot.hook.setPosition(0.45);
        }

        else if(gamepad2.x){ //in
            robot.arm.setPosition(0.99);
        }

        else if(gamepad2.y){ //out
            robot.arm.setPosition(0.01);
        }

        else if(gamepad2.left_bumper){
            robot.hook.setPosition(0.8);
        }

        if(gamepad2.right_bumper){
            robot.latch.setPosition(0);
        }

        if(gamepad2.dpad_up){
            robot.winch.setPower(0.5);
        }
        else if(gamepad2.dpad_down){
            robot.winch.setPower(-0.5);
        }
        else{
            robot.winch.setPower(0);
        }

        // positive down negative up
        if(!robot.touch.getState()) {
            if (gamepad2.left_stick_y != 0) {
                robot.slide.setPower(-Math.abs(gamepad2.left_stick_y));
            }
            else{
                robot.slide.setPower(0);
            }
        }
        else{
            if (gamepad2.left_stick_y != 0) {
                robot.slide.setPower(gamepad2.left_stick_y);
            }
            else{
                robot.slide.setPower(0);
            }
        }


    }

    //----------------------------------------------------------------------------------------------
    // Telemetry Configuration
    //----------------------------------------------------------------------------------------------

    void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gravity  = imu.getGravity();
        }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

        telemetry.addLine()
                .addData("grvty", new Func<String>() {
                    @Override public String value() {
                        return gravity.toString();
                    }
                })
                .addData("mag", new Func<String>() {
                    @Override public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(gravity.xAccel*gravity.xAccel
                                        + gravity.yAccel*gravity.yAccel
                                        + gravity.zAccel*gravity.zAccel));
                    }
                });
    }

    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}
