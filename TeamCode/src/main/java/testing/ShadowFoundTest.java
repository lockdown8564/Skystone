package testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.Range;

/**
 * This program is the main teleop for FTC 8564 as of 1/2/20.
 * Driver 1 has foundation grippers and the mecanum drivetrain, while
 * Driver 2 has the arm, linear slides, intake, and the rest of the scoring
 * mechanisms.
 *
 * @author William Trang
 * @version 5.2 1/2/20
 */

@TeleOp(name="found servo test",group="test")
public class ShadowFoundTest extends OpMode {
    private ShadowTestHardware robot = new ShadowTestHardware();
    private DriveSpeed driveSpeed;
    private ShadowTestHardware.DriveDirection driveDirection = ShadowTestHardware.DriveDirection.FORWARD;
    private Hopper hopper;

    private double forward, strafe, turn;
    private double deadzone;
    private double maxSpeed;

    private enum DriveSpeed{
        FAST,
        SLOW
    }

    /**
     Whether or not the hopper contains a stone
     */
    private enum Hopper{
        /**
         * has a stone
         */
        TRUE,

        /**
         * no stone
         */
        FALSE
    }

    @Override
    public void init(){
        hopper = Hopper.FALSE;
        driveSpeed = DriveSpeed.FAST;
        deadzone = 0.1;
        forward = 0;
        strafe = 0;
        turn = 0;
        maxSpeed = 0.8;

        robot.init(hardwareMap);
        robot.touch.setMode(DigitalChannel.Mode.INPUT);
    }

    @Override
    public void init_loop(){}
    @Override
    public void start(){}
    @Override
    public void loop(){
        if(gamepad1.a){
            robot.found1.setPosition(0.75);
            robot.found2.setPosition(0.75);
        }
        else if(gamepad1.b){
            robot.found1.setPosition(0.25);
            robot.found2.setPosition(0.25);
        }
    }

    @Override
    public void stop(){
        robot.stopMotors();
    }
}
