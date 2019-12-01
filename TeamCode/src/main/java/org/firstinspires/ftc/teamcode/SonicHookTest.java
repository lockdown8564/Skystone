package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcontroller.internal.FtcOpModeRegister;

/**
 * testing hook of intake night before lm1, 12/1/19
 */

@TeleOp(name="hook test",group="test")
public class SonicHookTest extends OpMode {
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
        if(gamepad2.x){
            robot.hook.setPosition(0.5);
        }
        else if(gamepad2.y){
            robot.hook.setPosition(0.9);
        }
    }
    @Override
    public void stop(){

    }
}
