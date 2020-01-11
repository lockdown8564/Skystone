package outreach;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import testing.ShadowTestHardware;

@Disabled
@TeleOp(name="shadow outreach teleop", group = "test")
public class ShadowOutreachTeleop extends OpMode {
    private ShadowTestHardware robot = new ShadowTestHardware();
    private ShadowTestHardware.DriveDirection driveDirection = ShadowTestHardware.DriveDirection.FORWARD;

    private double flPower, frPower, blPower, brPower = 0;
    private double forward, strafe, turn = 0;
    private double deadzone = 0.1;
    private double maxSpeed = 1;
    private double sign = 1;
    private double num = 0.1;

    private enum DriveSpeed{
        FAST,
        SLOW
    }

    /**
     Whether or not the hopper contains a stone
     */
    private enum Hopper{
        /**
         * has a stone
         */
        TRUE,

        /**
         * no stone
         */
        FALSE
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
        if(gamepad1.y){
            driveDirection = ShadowTestHardware.DriveDirection.FORWARD;
        }

        else if(gamepad1.x){
            driveDirection = ShadowTestHardware.DriveDirection.REVERSE;
        }

        if(Math.abs(gamepad1.left_stick_y) > deadzone){
            forward = gamepad1.left_stick_y;
        }
        else{
            forward = 0;
        }

        if(Math.abs(gamepad1.left_stick_x) > deadzone){
            strafe = gamepad1.left_stick_x;
        }
        else{
            strafe = 0;
        }

        if(Math.abs(gamepad1.right_stick_x) > deadzone){
            turn = gamepad1.right_stick_x;
        }
        else{
            turn = 0;
        }

        robot.mecanumDrive(forward, strafe, turn, driveDirection, 0.7);
    }

    @Override
    public void stop(){
        robot.stopMotors();
    }
    
}
