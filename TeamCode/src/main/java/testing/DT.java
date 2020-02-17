package testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class DT extends OpMode {
    private ShadowTestHardware robot = new ShadowTestHardware();
    private IntakeMode intakeMode = IntakeMode.FAST;
    private DriveMode driveMode = DriveMode.TANK;

    private enum DriveMode{
        ARCADE,
        TANK
    }
    private enum IntakeMode{
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
        if (gamepad1.a) {
            driveMode = DriveMode.ARCADE;
        } else if (gamepad1.b) {
            driveMode = DriveMode.TANK;
        }


        if (gamepad2.a) {
            intakeMode = IntakeMode.SLOW;
        } else if (gamepad2.b) {
            intakeMode = IntakeMode.FAST;
        }

        robot.driveSetPower(gamepad1.left_stick_y, gamepad1.right_stick_y, gamepad1.left_stick_y, gamepad1.right_stick_y);

        //switch based on selected drive mode
        switch(driveMode){
            case TANK:{
                //set left motors = left stick and right motors = right stick
                double lPower = gamepad1.left_stick_y;
                double rPower = gamepad1.right_stick_y;
                robot.driveSetPower(lPower,rPower, lPower, rPower);
                break;
            }
            case ARCADE:{
                double drivePower = gamepad1.left_stick_y; //forward and back on left stick y
                double turnPower = gamepad1.right_stick_x; //turn on right stick x
                robot.driveSetPower(drivePower-turnPower,drivePower+turnPower,
                        drivePower-turnPower, drivePower + turnPower);
                break;
            }
        }

        //switch based on desired intake mode
        switch(intakeMode) {
            case FAST: {
                robot.lIntake.setPower(gamepad2.left_stick_y);
                robot.rIntake.setPower(gamepad2.left_stick_y);
                break;
            }
            case SLOW: {
                robot.lIntake.setPower(gamepad2.left_stick_y * .5);
                robot.rIntake.setPower(gamepad2.left_stick_y * .5);
                break;
            }
        }
    }
    @Override
    public void stop(){

    }
}
