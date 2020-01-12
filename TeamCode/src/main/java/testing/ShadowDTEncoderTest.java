package testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Disabled
@TeleOp(name = "encoder test", group = "test")
public class ShadowDTEncoderTest extends OpMode {
    ShadowTestHardware robot = new ShadowTestHardware();

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
        telemetry.addData("fl:",robot.frontLeft.getCurrentPosition());
        telemetry.addData("bl:",robot.backLeft.getCurrentPosition());
        telemetry.addData("fr:",robot.frontRight.getCurrentPosition());
        telemetry.addData("br:",robot.backRight.getCurrentPosition());
        telemetry.update();
    }

    @Override
    public void stop(){

    }
}
