package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcontroller.internal.FtcOpModeRegister;

/**
 * test program for 6 wheel intake built on 9/15/19
 */

@TeleOp(name="color test",group="test")
public class SonicIntakeTest1 extends OpMode {
    private SonicHardware robot = new SonicHardware();
    public void init(){
        robot.init(hardwareMap);
        robot.foundation.setPosition(0.5);
    }
    @Override
    public void init_loop(){ }
    @Override
    public void start(){ }
    @Override
    public void loop(){
        telemetry.addData("Clear", robot.skystone.alpha());
        telemetry.addData("Red  ", robot.skystone.red());
        telemetry.addData("Green", robot.skystone.green());
        telemetry.addData("Blue ", robot.skystone.blue());

        if(gamepad1.x){
            robot.foundation.setPosition(0.6);
        }
        else if(gamepad1.y){
            robot.foundation.setPosition(0.4);
        }
        else if(gamepad1.a){
            robot.foundation.setPosition(0.3);
        }
        else if(gamepad1.b){
            robot.foundation.setPosition(0.7);
        }
        else if(gamepad1.left_bumper){
            robot.foundation.setPosition(0.2);
        }
        else if(gamepad1.right_bumper){
            robot.foundation.setPosition(0.8);
        }
    }
    @Override
    public void stop(){

    }
}
