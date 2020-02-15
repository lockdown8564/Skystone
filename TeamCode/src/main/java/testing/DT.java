package testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class DT extends OpMode {
    private ShadowTestHardware robot = new ShadowTestHardware();

    @Override
    public void init(){
        //initialize hardware
        robot.init(hardwareMap);

        //set zero power behaviors of motors
        robot.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }
    @Override
    public void init_loop(){ }
    @Override
    public void start(){ }
    @Override
    public void loop(){
        //set left motors to left stick and right motors to right stick
        robot.frontLeft.setPower(gamepad1.left_stick_y);
        robot.backLeft.setPower(gamepad1.left_stick_y);
        robot.frontRight.setPower(gamepad1.right_stick_y);
        robot.backRight.setPower(gamepad1.right_stick_y);
    }
    @Override
    public void stop(){

    }
}
