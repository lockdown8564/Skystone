package testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Teleop test to see if our Neverest encoders were working.
 * Returned telemetry about the encoder position of the motor.
 *
 * @version 1.0
 * @see ShadowNRTest
 */
@Disabled
@TeleOp(name = "nr test", group = "test")
public class ShadowNRTeleTest extends OpMode {
    DcMotor motor, bl = null;

    @Override
    public void init(){
        motor = hardwareMap.get(DcMotor.class,"motor");
        bl = hardwareMap.get(DcMotor.class,"bl");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void init_loop(){}

    @Override
    public void start(){}

    @Override
    public void loop(){
        motor.setPower(gamepad1.left_stick_y);
        telemetry.addData("Encoder:",motor.getCurrentPosition());
        telemetry.addData("Encoder BR:",bl.getCurrentPosition());
        telemetry.update();
    }

    @Override
    public void stop(){

    }
}
