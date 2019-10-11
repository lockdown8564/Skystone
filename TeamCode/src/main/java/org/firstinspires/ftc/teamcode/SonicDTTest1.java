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
    DriveMode driveMode = DriveMode.TANK;
    IntakeMode intakeMode = IntakeMode.FAST;
    private enum DriveMode{
        ARCADE,
        TANK
    }
    private enum IntakeMode{
        FAST,
        SLOW
    }
    @Override
    public void init(){
        robot.init(hardwareMap);
        robot.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }
    @Override
    public void init_loop(){}
    @Override
    public void start(){}
    @Override
    public void loop(){
        if(gamepad1.a){
            driveMode = DriveMode.ARCADE;
        }
        else if(gamepad1.b){
            driveMode = DriveMode.TANK;
        }

        if(gamepad2.a){
            intakeMode = IntakeMode.SLOW;
        }
        else if(gamepad2.b){
            intakeMode = IntakeMode.FAST;
        }

        robot.driveSetPower(gamepad1.left_stick_y,gamepad1.right_stick_y);

        //TODO: clean up with methods
        switch(driveMode){
            case TANK:{
                double lPower = gamepad1.left_stick_y*.5;
                double rPower = gamepad1.right_stick_y*.5;
                robot.driveSetPower(lPower,rPower);
                break;
            }
            case ARCADE:{
                double drivePower = gamepad1.left_stick_y;
                double turnPower = gamepad1.right_stick_x;
                robot.driveSetPower(drivePower-turnPower,drivePower+turnPower);
                break;
            }
        }

        switch(intakeMode){
            case FAST:{
                robot.lIntake.setPower(gamepad2.left_stick_y);
                robot.rIntake.setPower(gamepad2.right_stick_y);
                break;
            }
            case SLOW:{
                robot.lIntake.setPower(gamepad2.left_stick_y*.5);
                robot.rIntake.setPower(gamepad2.right_stick_y*.5);
                break;
            }
        }
    }
    @Override
    public void stop(){
        robot.stopMotors();
    }
}
