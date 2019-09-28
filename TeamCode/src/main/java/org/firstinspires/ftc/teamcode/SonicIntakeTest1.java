package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * test program for 6 wheel intake built on 9/15/19
 */

@TeleOp(name="intake test",group="test")
public class SonicIntakeTest1 extends OpMode {
    private DcMotor one,two = null;
    public void init(){
        one = hardwareMap.dcMotor.get("one");
        two = hardwareMap.dcMotor.get("two");
    }
    @Override
    public void init_loop(){ }
    @Override
    public void start(){ }
    @Override
    public void loop(){
        one.setPower(gamepad1.left_stick_y);
        two.setPower(gamepad1.right_stick_y);
    }
    @Override
    public void stop(){
        one.setPower(0);
        two.setPower(0);
    }
}
