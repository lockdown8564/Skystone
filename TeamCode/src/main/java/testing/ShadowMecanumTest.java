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
    private int slideInitPosition = 0;
    private int swingInitPosition = 0;
    private int level = 1;
    private int levelVar = 0;

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
        slideInitPosition = robot.slide.getCurrentPosition();
        swingInitPosition = robot.swing.getCurrentPosition();
    }

    @Override
    public void init_loop(){}
    @Override
    public void start(){}
    @Override
    public void loop(){
        if(gamepad1.right_trigger != 0){
            driveSpeed = DriveSpeed.SLOW;
            maxSpeed = 0.3;
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

        if(gamepad1.a){
            robot.gripFoundation();
        }

        else if(gamepad1.b){
            robot.releaseFoundation();
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
            if(driveDirection == ShadowTestHardware.DriveDirection.FORWARD){
                turn = gamepad1.right_stick_x;
            }
            else{
                turn = -gamepad1.right_stick_x;
            }
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

        //scoring
        /*if(gamepad2.x){
            switch(level){
                case 1: {
                    levelVar = -100;
                    break;
                }
                case 2: {
                    levelVar = 100;
                    break;
                }
                case 3: {
                    levelVar = 200;
                    break;
                }
                case 4: {
                    levelVar = 300;
                    break;
                }
                case 5: {
                    levelVar = 400;
                    break;
                }
                case 6: {
                    levelVar = 500;
                    break;
                }
            }

            robot.grip.setPosition(0.8);
            num = 0.16;

            robot.slide.setTargetPosition(slideInitPosition + 500);
            robot.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.slide.setPower(0.4);
            while(robot.slide.isBusy()){
            }

            robot.stopMotors();
            robot.slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            /*robot.swing.setTargetPosition(swingInitPosition + 120);
            robot.swing.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.swing.setPower(-0.4);
            while(robot.swing.isBusy()){
            }

            robot.stopMotors();
            robot.swing.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            robot.slide.setTargetPosition(slideInitPosition + levelVar);
            robot.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.slide.setPower(0.4);
            while(robot.slide.isBusy()){
            }

            robot.stopMotors();

            robot.slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        //original
        else if(gamepad2.y){
            robot.grip.setPosition(0);
            num = 0.1;

            /*robot.swing.setTargetPosition(swingInitPosition);
            robot.swing.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.swing.setPower(0.4);
            while(robot.swing.isBusy()){
            }

            robot.stopMotors();
            robot.swing.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            robot.slide.setTargetPosition(slideInitPosition);
            robot.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.slide.setPower(0.4);
            while(robot.slide.isBusy()){
            }

            robot.stopMotors();
            robot.slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }*/

        if (gamepad2.left_stick_y != 0) {
            robot.slide.setPower(-gamepad2.left_stick_y);
        }

        else {
            robot.slide.setPower(0.0001);
        }

        if(gamepad2.a){
            robot.grip.setPosition(0);
            num = 0.1;
        }

        else if(gamepad2.b){
            robot.grip.setPosition(0.8);
            num = 0.16;
        }

        if(gamepad2.right_stick_y != 0) {
            robot.swing.setPower(gamepad2.right_stick_y);
            sign = Math.signum(gamepad2.right_stick_y);
        }

        else{
            robot.swing.setPower(0);
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

        if(gamepad2.right_bumper){
            level++;
        }
        else if(gamepad2.right_trigger != 0){
            level--;
        }

        telemetry.addData("Hopper:",hopper.toString());
        telemetry.addData("Swing Pos:",robot.swing.getCurrentPosition());
        telemetry.addData("Slide Pos:",robot.slide.getCurrentPosition());
        telemetry.update();
    }

    @Override
    public void stop(){
        robot.stopMotors();
    }
}
