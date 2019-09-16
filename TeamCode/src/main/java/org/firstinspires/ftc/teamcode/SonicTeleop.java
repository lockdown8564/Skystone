package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Disabled
@TeleOp(name="teletest",group="one")
public class SonicTeleop extends OpMode {
    SonicHardware robot = new SonicHardware();
    public void init() {
        robot.init(hardwareMap);
    }
    @Override
    public void init_loop(){}
    @Override
    public void start(){}
    @Override
    public void loop() {
        //TODO: use enums to clean up
        int driveMode = 1;
        if(gamepad1.a){
           driveMode = 2;
        }
        else if(gamepad1.b){
            driveMode = 1;
        }

        //TODO: clean up with methods
        switch(driveMode){
            case 1:{
                final double leftPowerTank = gamepad1.left_stick_y;
                final double rightPowerTank = gamepad1.right_stick_y;
                robot.frontLeft.setPower(leftPowerTank);
                robot.backLeft.setPower(leftPowerTank);
                robot.frontRight.setPower(rightPowerTank);
                robot.backRight.setPower(rightPowerTank);
            }
            case 2:{
                final double drivePower = gamepad1.left_stick_y;
                final double turnPower = gamepad1.right_stick_x;
                robot.frontLeft.setPower(drivePower-turnPower);
                robot.backLeft.setPower(drivePower-turnPower);
                robot.frontRight.setPower(drivePower+turnPower);
                robot.backRight.setPower(drivePower+turnPower);
            }
        }
    }
    @Override
    public void stop(){
        robot.stopMotors();
    }
}
