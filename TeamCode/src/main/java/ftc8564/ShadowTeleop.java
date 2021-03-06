package ftc8564;

import android.content.Context;
import android.media.MediaPlayer;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.R;

/**
 * This program is the main teleop for FTC 8564 as of 3/9/20.
 * Driver 1 has foundation grippers and the mecanum drivetrain, while
 * Driver 2 has the arm, linear slides, intake, and the rest of the scoring
 * mechanisms.
 *
 * @author William Trang
 * @version 5.2 3/9/20
 */

@TeleOp (name = "shadow teleop", group = "ftc8564")
public class ShadowTeleop extends OpMode {
    private ShadowHardware robot = new ShadowHardware();
    private DriveSpeed driveSpeed;
    private ShadowHardware.DriveDirection driveDirection = ShadowHardware.DriveDirection.FORWARD;
    private Hopper hopper;

    private double forward, strafe, turn;
    private double deadzone;
    private double maxSpeed;

    private MediaPlayer fancyPlayer;
    private MediaPlayer tokyoDriftPlayer;
    private Context app = hardwareMap.appContext;

    private enum DriveSpeed{
        FAST,
        SLOW,
        SUPER_SLOW
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
        hopper = Hopper.FALSE;
        driveSpeed = DriveSpeed.FAST;
        deadzone = 0.08;
        forward = 0;
        strafe = 0;
        turn = 0;
        maxSpeed = 0.8;

        fancyPlayer = MediaPlayer.create(app, R.raw.fancy);
        tokyoDriftPlayer = MediaPlayer.create(app, R.raw.tokyodrift);
        robot.init(hardwareMap);
    }

    @Override
    public void init_loop(){}
    @Override
    public void start(){}
    @Override
    public void loop(){
        if(gamepad1.left_trigger != 0){
            tokyoDriftPlayer.start();
            driveSpeed = DriveSpeed.SUPER_SLOW;
            maxSpeed = 0.2;
        }

        else if(gamepad1.right_trigger != 0){
            tokyoDriftPlayer.start();
            driveSpeed = DriveSpeed.SLOW;
            maxSpeed = 0.4;
        }

        else {
            driveSpeed = DriveSpeed.FAST;
            maxSpeed = 1;
            tokyoDriftPlayer.release();
        }

        if(gamepad2.dpad_down) { //retract
            robot.yeet1.setPower(-0.5);
            robot.yeet2.setPower(-0.5);
        }

        else if (gamepad2.dpad_up) { //extend
            robot.yeet1.setPower(0.5);
            robot.yeet2.setPower(0.5);
        }

        else {
            robot.yeet1.setPower(0);
            robot.yeet2.setPower(0);
        }

        if(gamepad1.y){
            driveDirection = ShadowHardware.DriveDirection.FORWARD;
        }

        else if(gamepad1.x){
            driveDirection = ShadowHardware.DriveDirection.REVERSE;
        }

        if(gamepad1.a){
            robot.gripFoundation();
        }

        else if(gamepad1.b){
            robot.releaseFoundation();
        }

        if(Math.abs(gamepad1.left_stick_y) > deadzone) {
            forward = -gamepad1.left_stick_y;
        }

        else {
            forward = 0;
        }

        if(Math.abs(gamepad1.left_stick_x) > deadzone) {
            strafe = gamepad1.left_stick_x;
        }

        else {
            strafe = 0;
        }

        if(Math.abs(gamepad1.right_stick_x) > deadzone) {
            if(driveDirection == ShadowHardware.DriveDirection.FORWARD) {
                turn = -gamepad1.right_stick_x;
            }
            else{
                turn = gamepad1.right_stick_x;
            }
        }
        else {
            turn = 0;
        }

        robot.mecanumDrive(forward, strafe, turn, driveDirection, maxSpeed);

        //down = in
        if(gamepad2.left_bumper) {
            robot.intakeSetPower(-0.8);
        }
        //up = out
        else if(gamepad2.left_trigger != 0){
            robot.intakeSetPower(0.8);
        }

        else if(gamepad2.right_bumper){
            robot.intakeSetPower(0.3);
        }

        else{
            robot.intakeSetPower(0);
        }

        if (gamepad2.left_stick_y != 0) {
            robot.slide.setPower(-gamepad2.left_stick_y);
        }

        else {
            robot.slide.setPower(0);
        }

        if(gamepad2.a){
            robot.grip.setPosition(0.6);
        }

        else if(gamepad2.b){
            robot.grip.setPosition(0.95);
        }

        if(gamepad2.right_stick_y != 0) {
            robot.swing.setPower(gamepad2.right_stick_y);
        }

        /*else if(gamepad2.y){
            while(robot.swing.getCurrentPosition() > -60){
                robot.swing.setPower(0.6);
            }

            robot.stopMotors();

            robot.swing.setTargetPosition(100);
            robot.swing.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.swing.setPower(-0.5);
            while(robot.swing.isBusy()){
            }

            robot.stopMotors();
            robot.driveSetMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }

        else if(gamepad2.x){
            while(robot.swing.getCurrentPosition() < -20){
                robot.swing.setPower(-0.6);
            }

            robot.swing.setTargetPosition(10);
            robot.swing.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.swing.setPower(-0.5);
            while(robot.swing.isBusy()){
            }

            robot.stopMotors();
            robot.driveSetMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }*/

        else{
            robot.swing.setPower(0);
        }

        if(gamepad2.left_trigger != 0){
            fancyPlayer.start();
        }
        else{
            fancyPlayer.release();
        }

        telemetry.addData("Swing:", robot.swing.getCurrentPosition());

        if(robot.color.red() > 155 && robot.color.green() > 135){
            hopper = Hopper.TRUE;
        }
        else {
            hopper = Hopper.FALSE;
        }
    }

    @Override
    public void stop(){
        robot.stopMotors();
    }
}
