package outreach;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import testing.ShadowTestHardware;

@TeleOp(name="outreach teleop", group = "test")
public class ShadowOutreachTeleop extends OpMode {
    private ShadowTestHardware robot = new ShadowTestHardware();
    private ShadowTestHardware.DriveDirection driveDirection = ShadowTestHardware.DriveDirection.FORWARD;

    private double forward, strafe, turn = 0;
    private double deadzone = 0.1;

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

        if(gamepad2.right_trigger != 0) {
            if(Math.abs(gamepad2.left_stick_y) > deadzone){
                forward = gamepad2.left_stick_y;
            }
            else{
                forward = 0;
            }

            if(Math.abs(gamepad2.left_stick_x) > deadzone){
                strafe = gamepad2.left_stick_x;
            }
            else{
                strafe = 0;
            }

            if(Math.abs(gamepad2.right_stick_x) > deadzone){
                turn = gamepad2.right_stick_x;
            }
            else{
                turn = 0;
            }

            robot.mecanumDrive(forward, strafe, turn, driveDirection, 1);
        }

        else if (gamepad1.left_stick_y != 0 || gamepad1.right_stick_x != 0 || gamepad1.left_stick_x != 0 || gamepad1.right_stick_y != 0){
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

        else {
            robot.stopMotors();
        }
    }

    @Override
    public void stop(){
        robot.stopMotors();
    }
    
}
