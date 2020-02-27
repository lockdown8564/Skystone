package testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class DT extends OpMode {
    private ShadowTestHardware robot = new ShadowTestHardware();
    private DriveMode driveMode = DriveMode.TANK;
    private DriveSpeed driveSpeed = DriveSpeed.FAST;
    private DriveDirection driveDirection = DriveDirection.FORWARD;
    private double lPower, rPower, drivePower, turnPower = 0;
    Servo arm, hook = null;

    private double num = 1;

    private enum DriveMode{
        ARCADE,
        TANK
    }

    private enum DriveSpeed{
        FAST,
        SLOW
    }

    private enum DriveDirection{
        FORWARD,
        REVERSE
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
        if(gamepad1.right_trigger != 0){ //hold down to enable slow
            driveSpeed = DriveSpeed.SLOW;
            num = 0.5;
        }

        else {
            driveSpeed = DriveSpeed.FAST;
            num = 1;
        }

        //switch drive direction
        if(gamepad1.y){
            driveDirection = DriveDirection.FORWARD;
        }
        else if(gamepad1.x){
            driveDirection = DriveDirection.REVERSE;
        }

        //switch based on drive mode
        switch(driveMode){
            case TANK:{ //tank selected
                switch(driveDirection) {
                    case REVERSE:
                        lPower = -gamepad1.left_stick_y;
                        rPower = -gamepad1.right_stick_y;
                        robot.driveSetPower(lPower * num, rPower * num
                        , lPower * num, rPower * num); //num is for slow and fast
                        break;
                    case FORWARD:
                        lPower = gamepad1.right_stick_y;
                        rPower = gamepad1.left_stick_y;
                        robot.driveSetPower(lPower * num, rPower * num
                                , lPower * num, rPower * num); //num is for slow and fast
                        break;
                }
                break;
            }
            case ARCADE:{//arcade selected
                switch(driveDirection) {
                    case REVERSE:
                        drivePower = -gamepad1.left_stick_y;
                        turnPower = gamepad1.right_stick_x;
                        robot.driveSetPower((drivePower-turnPower)*num,(drivePower+turnPower)*num,
                                (drivePower-turnPower)*num, (drivePower+turnPower)*num); //num is for slow and fast
                        break;
                    case FORWARD:
                        drivePower = gamepad1.left_stick_y;
                        turnPower = gamepad1.right_stick_x;
                        robot.driveSetPower((drivePower-turnPower)*num,(drivePower+turnPower)*num,
                                (drivePower-turnPower)*num, (drivePower+turnPower)*num); //num is for slow and fast
                        break;
                }
                break;
            }
        }


        if(gamepad2.left_trigger != 0){ //in
            arm.setPosition(1);
        }
        else if(gamepad2.right_trigger != 0){ //score
            arm.setPosition(0);
        }

        if(robot.touch.getState()) { //if touch is pressed,
            if (gamepad2.left_stick_y != 0) {
                robot.slide.setPower(-Math.abs(gamepad2.left_stick_y));
            }
            else{
                robot.slide.setPower(0);
            }
        }
        else{
            if (gamepad2.left_stick_y != 0) {
                robot.slide.setPower(gamepad2.left_stick_y);
            }
            else{
                robot.slide.setPower(0);
            }
        }

        //right trigger for slow intake mode
        if(gamepad2.right_trigger != 0){
            if(gamepad2.right_stick_y > 0) { //down = in
                robot.intakeSetPower(-0.5);
            }
            else if(gamepad2.right_stick_y < 0) { //up = out
                robot.intakeSetPower(0.5);
            }
        }
        else if(gamepad2.right_stick_y > 0) { //down = in
            robot.intakeSetPower(-1);
        }
        else if(gamepad2.right_stick_y < 0) { //up = out
            robot.intakeSetPower(1);
        }
        else {
            robot.intakeSetPower(0);
        }

        if(gamepad2.b){ //don't want to move the hook and arm at the same time
            robot.grip.setPosition(0.45);
        }

        else if(gamepad2.x){ //in
            arm.setPosition(0.99);
        }

        else if(gamepad2.y){ //out
            arm.setPosition(0.01);
        }



    }
    @Override
    public void stop(){

    }
}
