package testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.Range;

/**
 * mecanum drivetrain test for shadow
 * created: 12/7/19
 * last updated: 1/2/20
 */

@TeleOp(name="shadow mec test",group="test")
public class ShadowMecanumTest extends OpMode {
    private ShadowTestHardware robot = new ShadowTestHardware();
    private DriveSpeed driveSpeed = DriveSpeed.FAST;
    private ShadowTestHardware.DriveDirection driveDirection = ShadowTestHardware.DriveDirection.FORWARD;
    private Hopper hopper = Hopper.FALSE;

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
        robot.touch.setMode(DigitalChannel.Mode.INPUT);
    }

    @Override
    public void init_loop(){}
    @Override
    public void start(){}
    @Override
    public void loop(){
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
        if(gamepad2.left_bumper) {
            robot.intakeSetPower(-1);
        }
        //up = out
        else if(gamepad2.left_trigger != 0){
            robot.intakeSetPower(1);
        }

        else{
            robot.intakeSetPower(0);
        }

        if(gamepad2.x){
            robot.gripFoundation();
        }
        else if(gamepad2.y){
            robot.releaseFoundation();
        }

        if (gamepad2.left_stick_y != 0) {
            robot.slide.setPower(-gamepad2.left_stick_y);
        }

        else {
            robot.slide.setPower(0.1);
        }

        if(gamepad2.a){
            robot.grip.setPosition(0);
            num = 0.1;
        }

        else if(gamepad2.b){
            robot.grip.setPosition(0.8);
            num = 0.18;
        }

        if(gamepad2.right_stick_y != 0) {
            robot.swing.setPower(gamepad2.right_stick_y * 0.5);
            sign = Math.signum(gamepad2.right_stick_y);
        }

        else{
            robot.swing.setPower(num * sign);
        }

        if (robot.touch.getState()) {
            telemetry.addData("Digital Touch", "Is Not Pressed");
        }

        else {
            telemetry.addData("Digital Touch", "Is Pressed");
        }

        if(robot.color.red() > 155 && robot.color.green() > 135){
            hopper = Hopper.TRUE;
        }
        else {
            hopper = Hopper.FALSE;
        }

        telemetry.addData("Hopper:",hopper.toString());
        telemetry.update();
    }

    @Override
    public void stop(){
        robot.stopMotors();
    }
}
