package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * first drivetrain test
 * 6 wheels made of wood 9/22/19
 */
@TeleOp(name="dt test",group="test")
public class SonicDTTest1 extends OpMode {
    private SonicHardware robot = new SonicHardware();
    private DriveMode driveMode = DriveMode.TANK;
    private DriveSpeed driveSpeed = DriveSpeed.FAST;
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
    @Override
    public void init(){
        robot.init(hardwareMap);
        robot.hook.setPosition(0);
        initialSlide = robot.arm.getCurrentPosition();
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
        else if(gamepad1.a && driveMode == DriveMode.ARCADE){
            driveMode = DriveMode.TANK;
        }

        if(gamepad1.right_bumper && driveSpeed == DriveSpeed.SLOW){
            driveSpeed = DriveSpeed.FAST;
            num = 1;
        }
        else if(gamepad1.right_bumper && driveSpeed == DriveSpeed.FAST){
            driveSpeed = DriveSpeed.SLOW;
            num = 0.5;
        }

        //TODO: clean up with methods
        switch(driveMode){
            case TANK:{
                double lPower = gamepad1.left_stick_y;
                double rPower = gamepad1.right_stick_y;
                robot.driveSetPower(lPower*num,rPower*num);
                break;
            }
            case ARCADE:{
                double drivePower = gamepad1.left_stick_y;
                double turnPower = gamepad1.right_stick_x;
                robot.driveSetPower((drivePower-turnPower)*num,(drivePower+turnPower)*num);
                break;
            }
        }

        //in
        if(gamepad2.a) {
            robot.intakeSetPower(-1);
        }
        //out
        else if(gamepad2.b){
            robot.intakeSetPower(1);
        }
        else{
            robot.intakeSetPower(0);
        }

        if(gamepad2.x){
            robot.latch.setPosition(0);
        }

        if(gamepad2.right_bumper){
            robot.hook.setPosition(0.4);
        }
        else if(gamepad2.left_bumper){
            robot.hook.setPosition(0);
        }

        // positive down negative up
        if(gamepad2.left_stick_y != 0) {
            robot.slide.setPower(gamepad2.left_stick_y);
        }
        else {
            robot.slide.setPower(0);
        }

        if(gamepad2.right_stick_y<0){
            robot.arm.setPower(gamepad2.right_stick_y * .15);
        }
        else{
            robot.arm.setPower(gamepad2.right_stick_y * .25);
        }

    }
    @Override
    public void stop(){
        robot.stopMotors();
    }
}
