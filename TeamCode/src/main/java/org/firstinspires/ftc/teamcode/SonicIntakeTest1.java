package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcontroller.internal.FtcOpModeRegister;

/**
 * test program for 6 wheel intake built on 9/15/19
 */

@Disabled
@TeleOp(name="color test",group="test")
public class SonicIntakeTest1 extends OpMode {
    private SonicHardware robot = new SonicHardware();
    public void init(){
        robot.init(hardwareMap);
    }
    @Override
    public void init_loop(){ }
    @Override
    public void start(){ }
    @Override
    public void loop(){

    }
    @Override
    public void stop(){

    }
}
