package testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.Range;

/**
 * Testing the linear slide on Shadow.
 * This happened the same day as us messing up our wiring lol.
 *
 * @author William Trang
 * @version 1.0 12/28/19
 * @see ShadowTeleopTest
 */

@Disabled
@TeleOp(name="shadow slide test",group="test")
public class ShadowSlideTest extends OpMode {
    private ShadowTestHardware robot = new ShadowTestHardware();

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
        if (gamepad1.left_stick_y != 0) {
            robot.slide.setPower(gamepad1.left_stick_y);
        }

        else {
            robot.slide.setPower(0);
        }

        if (gamepad1.x){
            robot.releaseFoundation();
        }

        else if (gamepad1.y) {
            robot.gripFoundation();
        }
    }

    @Override
    public void stop(){
        robot.stopMotors();
    }
}
