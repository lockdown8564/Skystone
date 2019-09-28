package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * first drivetrain test
 * 6 wheels made of wood 9/22/19
 */
@TeleOp(name="dt test",group="test")
public class SonicDTTest1 extends OpMode {
    private SonicHardware robot = new SonicHardware();
    private enum DriveMode{
        ARCADE,
        TANK
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
        DriveMode driveMode = DriveMode.TANK;
        if(gamepad1.a){
            driveMode = DriveMode.ARCADE;
        }
        else if(gamepad1.b){
            driveMode = DriveMode.TANK;
        }

        //TODO: clean up with methods
        switch(driveMode){
            case TANK:{
                double lPower = gamepad1.left_stick_y;
                double rPower = gamepad1.right_stick_y;
                robot.driveSetPower(lPower,rPower);
            }
            case ARCADE:{
                double drivePower = gamepad1.left_stick_y;
                double turnPower = gamepad1.right_stick_x;
                robot.driveSetPower(drivePower-turnPower,drivePower+turnPower);
            }
        }
    }
    @Override
    public void stop(){
        robot.stopMotors();
    }
}
