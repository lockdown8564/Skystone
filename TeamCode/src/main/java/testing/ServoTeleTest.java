package testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.Range;

@Disabled
@TeleOp(name="servo test tele",group="test")
public class ServoTeleTest extends OpMode {
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
            robot.grip.setPosition(0.95);
        }
        else if(gamepad1.b){
            robot.grip.setPosition(0.6);
        }

        telemetry.addData("Position:", robot.grip.getPosition());
        telemetry.update();
    }

    @Override
    public void stop(){
        robot.stopMotors();
    }
}
