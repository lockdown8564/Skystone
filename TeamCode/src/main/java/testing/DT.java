package testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class DT extends OpMode {
    private ShadowTestHardware robot = new ShadowTestHardware();
    private DriveMode driveMode = DriveMode.TANK;
    private DriveSpeed driveSpeed = DriveSpeed.FAST;

    private double num = 1;

    private enum DriveMode{
        ARCADE,
        TANK
    }

    private enum DriveSpeed{
        FAST,
        SLOW
    }

    @Override
    public void init() {
        //initialize hardware
        robot.init(hardwareMap);
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {
        //switch drive mode with button presses
        if (gamepad1.a) {
            driveMode = DriveMode.ARCADE;
        }
        else if (gamepad1.b) {
            driveMode = DriveMode.TANK;
        }

        if(gamepad1.right_bumper){
            driveSpeed = DriveSpeed.FAST;
            num = 1; //speed factor
        }
        else if(gamepad1.right_trigger != 0){
            driveSpeed = DriveSpeed.SLOW;
            num = 0.5; //speed factor
        }

        //switch based on selected drive mode
        switch(driveMode){
            case TANK:{
                //set left motors = left stick and right motors = right stick
                double lPower = gamepad1.left_stick_y;
                double rPower = gamepad1.right_stick_y;
                robot.driveSetPower(lPower*num,rPower*num, lPower*num, rPower*num);
                break;
            }
            case ARCADE:{
                double drivePower = gamepad1.left_stick_y; //forward and back on left stick y
                double turnPower = gamepad1.right_stick_x; //turn on right stick x
                robot.driveSetPower(drivePower-turnPower * num,drivePower+turnPower * num,
                        drivePower-turnPower * num, drivePower + turnPower * num);
                break;
            }
        }

        if(gamepad2.a) { //in
            robot.intakeSetPower(-1);
        }
        else if(gamepad2.b) { //out
            robot.intakeSetPower(1);
        }
        else {
            robot.intakeSetPower(0);
        }

        if(gamepad2.right_stick_y<0){ //moving out
            robot.swing.setPower(gamepad2.right_stick_y * .15);
        }
        else{ //moving back in
            robot.swing.setPower(gamepad2.right_stick_y * .15);
        }

        if(gamepad2.right_bumper){ //up position, ungripped
            robot.grip.setPosition(0.4);
        }
        else if(gamepad2.left_bumper){ //down position, gripped
            robot.grip.setPosition(0);
        }

        //kick out intake
        if(gamepad1.x){
            robot.latch.setPosition(1);
        }

        // positive down negative up
        robot.slide.setPower(gamepad2.left_stick_y);
    }
    @Override
    public void stop(){

    }
}
