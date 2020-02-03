package ftc8564;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import testing.ShadowTestHardware;

@TeleOp (name = "shadow teleop", group = "ftc8564")
@Disabled
public class ShadowTeleop extends OpMode {
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


    }

    @Override
    public void stop(){
        robot.stopMotors();
    }
}
