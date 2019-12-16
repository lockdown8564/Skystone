package testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * first drivetrain test
 * 6 wheels made of wood 9/22/19
 * Last Updated: 11/17/19
 */
@TeleOp(name="dt test",group="test")
public class SonicDTTest1 extends OpMode {
    private SonicTestHardware robot = new SonicTestHardware();
    private DriveMode driveMode = DriveMode.TANK;
    private DriveSpeed driveSpeed = DriveSpeed.FAST;
    private DriveDirection driveDirection = DriveDirection.FORWARD;

    private double lPower, rPower, drivePower, turnPower = 0;
    private double num = 1;
    private int initialSlide = 0;

    private enum DriveMode{
        ARCADE,
        TANK,
    }
    private enum DriveSpeed{
        FAST,
        SLOW
    }
    private enum DriveDirection{
        FORWARD,
        REVERSE
    }

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
        if(gamepad1.a && driveMode == DriveMode.TANK){
            driveMode = DriveMode.ARCADE;
        }

        else if(gamepad1.b && driveMode == DriveMode.ARCADE){
            driveMode = DriveMode.TANK;
        }

        if(gamepad1.right_trigger != 0){
            driveSpeed = DriveSpeed.SLOW;
            num = 0.5;
        }

        else {
            driveSpeed = DriveSpeed.FAST;
            num = 1;
        }

        if(gamepad1.y){
            driveDirection = DriveDirection.FORWARD;
        }

        else if(gamepad1.x){
            driveDirection = DriveDirection.REVERSE;
        }

        //TODO: clean up with methods
        switch(driveMode){
            case TANK:{
                switch(driveDirection) {
                    case REVERSE:
                        lPower = -gamepad1.left_stick_y;
                        rPower = -gamepad1.right_stick_y;
                        robot.driveSetPower(lPower * num, rPower * num);
                        break;
                    case FORWARD:
                        lPower = gamepad1.right_stick_y;
                        rPower = gamepad1.left_stick_y;
                        robot.driveSetPower(lPower * num, rPower * num);
                        break;
                }
                break;
            }
            case ARCADE:{
                switch(driveDirection) {
                    case REVERSE:
                        drivePower = -gamepad1.left_stick_y;
                        turnPower = gamepad1.right_stick_x;
                        robot.driveSetPower((drivePower+turnPower)*num,(drivePower-turnPower)*num);
                        break;
                    case FORWARD:
                        drivePower = gamepad1.left_stick_y;
                        turnPower = gamepad1.right_stick_x;
                        robot.driveSetPower((drivePower + turnPower)*num,(drivePower - turnPower)*num);
                        break;
                }
                break;
            }
        }

        //down = in
        if(gamepad2.right_trigger != 0){
            if(gamepad2.right_stick_y > 0) {
                robot.intakeSetPower(-0.5);
            }
            //up = out
            else if(gamepad2.right_stick_y < 0){
                robot.intakeSetPower(0.5);
            }
        }
        else if(gamepad2.right_stick_y > 0) {
            robot.intakeSetPower(-1);
        }
        //up = out
        else if(gamepad2.right_stick_y < 0){
            robot.intakeSetPower(1);
        }

        else{
            robot.intakeSetPower(0);
        }

        if(gamepad2.a){
            robot.hook.setPosition(1);
        }

        else if(gamepad2.b){
            robot.hook.setPosition(0.45);
        }

        else if(gamepad2.x){ //in
            robot.arm.setPosition(0.99);
        }

        else if(gamepad2.y){ //out
            robot.arm.setPosition(0.01);
        }

        else if(gamepad2.left_bumper){
            robot.hook.setPosition(0.8);
        }

        if(gamepad2.right_bumper){
            robot.latch.setPosition(0);
        }

        if(gamepad2.dpad_up){
            robot.winch.setPower(0.5);
        }
        else if(gamepad2.dpad_down){
            robot.winch.setPower(-0.5);
        }
        else{
            robot.winch.setPower(0);
        }

        // positive down negative up
        if(!robot.touch.getState()) {
            if (gamepad2.left_stick_y != 0) {
                robot.slide.setPower(-Math.abs(gamepad2.left_stick_y));
            }
            else{
                robot.slide.setPower(0);
            }
        }
        else{
            if (gamepad2.left_stick_y != 0) {
                robot.slide.setPower(gamepad2.left_stick_y);
            }
            else{
                robot.slide.setPower(0);
            }
        }

    }
    @Override
    public void stop(){
        robot.stopMotors();
    }
}
