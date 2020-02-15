package testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.Range;

/**
 * Test teleop for shadow. Robot wasn't working so I had to try something
 * LOL. I thought it might've been the program but it turns out that it
 * wasn't, just something with the config and wiring.
 *
 * @author William Trang
 * @version 1.0 12/28/19
 */

@Disabled
@TeleOp(name="shadow tele test",group="test")
public class ShadowTeleopTest extends OpMode {
    private ShadowTestHardware robot = new ShadowTestHardware();
    private DriveSpeed driveSpeed = DriveSpeed.FAST;
    private ShadowTestHardware.DriveDirection driveDirection = ShadowTestHardware.DriveDirection.FORWARD;

    private double flPower, frPower, blPower, brPower = 0;
    private double forward, strafe, turn = 0;
    private double deadzone = 0.1;
    private double maxSpeed = 1;


    private enum DriveSpeed{
        FAST,
        SLOW
    }

    @Override
    public void init(){
        robot.init(hardwareMap);
        robot.touch.setMode(DigitalChannel.Mode.INPUT);
    }

    @Override
    public void init_loop(){}
    @Override
    public void start(){}
    @Override
    public void loop(){
        if (gamepad2.left_stick_y != 0) {
            robot.slide.setPower(gamepad2.left_stick_y);
        }

        else {
            robot.slide.setPower(0);
        }

        if(gamepad1.right_trigger != 0){
            driveSpeed = DriveSpeed.SLOW;
            maxSpeed = 0.5;
        }

        else {
            driveSpeed = DriveSpeed.FAST;
            maxSpeed = 1;
        }

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

        robot.mecanumDrive(forward, strafe, turn, driveDirection, maxSpeed);

        //down = in
        if(gamepad2.right_stick_y > 0) {
            robot.intakeSetPower(-1);
        }
        //up = out
        else if(gamepad2.right_stick_y < 0){
            robot.intakeSetPower(1);
        }

        else{
            robot.intakeSetPower(0);
        }

        if(gamepad2.x){
            robot.found1.setPosition(0);
            robot.found2.setPosition(0);
        }
        else if(gamepad2.y){
            robot.found1.setPosition(1);
            robot.found2.setPosition(1);
        }

        if (robot.touch.getState()) {
            telemetry.addData("Digital Touch", "Is Not Pressed");
        }

        else {
            telemetry.addData("Digital Touch", "Is Pressed");
        }

        telemetry.update();

    }

    @Override
    public void stop(){
        robot.stopMotors();
    }
}
