package outreach;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import rip_sonic.SonicHardware;

/**
 * teleop for Sonic, 2019-2020 skystone robot
 * last updated 12/6/19
 * changes infrequently, only after files are tested
 */
@TeleOp(name="outreach teleop",group="test")
public class SonicOutreachTeleop extends OpMode {
    private SonicHardware robot = new SonicHardware();
    private DriveMode driveMode = DriveMode.ARCADE;
    private DriveSpeed driveSpeed = DriveSpeed.FAST;
    private DriveDirection driveDirection = DriveDirection.FORWARD;

    private double lPower, rPower, drivePower, turnPower = 0;
    private double num = 1;

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

    @Override
    public void init(){
        robot.init(hardwareMap);
    }

    @Override
    public void init_loop(){}
    @Override
    public void start(){}
    @Override
    public void loop(){
        if(gamepad1.right_trigger != 0){
            driveSpeed = DriveSpeed.SLOW;
            num = 0.5;
        }

        else {
            driveSpeed = DriveSpeed.FAST;
            num = 1;
        }

        if(gamepad2.right_trigger != 0) {
            drivePower = gamepad2.left_stick_y;
            turnPower = gamepad2.right_stick_x;
            robot.driveSetPower((drivePower + turnPower) * num * .7, (drivePower - turnPower) * num * .7);
        }

        else if (gamepad1.left_stick_y != 0 || gamepad1.right_stick_x != 0 || gamepad1.left_stick_x != 0 || gamepad1.right_stick_y != 0){
            drivePower = gamepad1.left_stick_y;
            turnPower = gamepad1.right_stick_x;
            robot.driveSetPower((drivePower + turnPower) * num * .7, (drivePower - turnPower) * num * .7);
        }

        else {
            robot.stopMotors();
        }
    }
    @Override
    public void stop(){
        robot.stopMotors();
    }
}
