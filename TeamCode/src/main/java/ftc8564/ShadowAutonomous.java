package ftc8564;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import ftclib.FtcMenu;
import ftclib.FtcChoiceMenu;
import ftclib.FtcValueMenu;
import hallib.HalDashboard;

/**
 * The one and only autonomous for Shadow (once I implement everything).
 * Implements FtcMenu for customization of autonomous programs.
 * Autonomous programs are developed in separate programs for ease.
 *
 * @author William Trang
 * @version 4.1 3/11/20
 * @see ftclib.FtcMenu
 * @see testing
 */

@Autonomous (name = "shadow auto", group = "ftc8564")
@Disabled
public class ShadowAutonomous extends LinearOpMode implements FtcMenu.MenuButtons {
    /* DECLARE OPMODE MEMBEERS */
    private ShadowHardware robot = new ShadowHardware();
    private OpenCvCamera webcam;
    private SSDetector detector;

    /* DECLARE VARIABLES FOR MENU */
    private Alliance alliance;
    private int delay;
    private StartingPos startpos;
    private Skystones skystones;
    private Skystone skystone;
    private Stones stones;
    private Park park;
    private Foundation foundation;

    private static HalDashboard dashboard;

    /**
     * our alliance for each match
     */
    private enum Alliance{
        /**
         * blue alliance side
         */
        BLUE,

        /**
         * red alliance side
         */
        RED
    }

    /**
     * starting position of robot in autonomous
     */
    private enum StartingPos{
        /**
         * foundation side one tile away from the side wall
         */
        FOUNDATION,

        /**
         * stone side along the side wall
         */
        STONE1,

        /**
         * stone side one tile away from the side wall
         */
        STONE2
    }

    /**
     * location of skystone during autonomous
     */
    private enum Skystone {
        /**
         * skystones are on the leftmost side
         */
        LEFT,

        /**
         * skystones are in the center
         */
        MIDDLE,

        /**
         * skystones are on the rightmost side
         */
        RIGHT
    }

    /**
     * number of skystones to attempt to score during autonomous
     */
    private enum Skystones{
        /**
         * attempt 0 skystones
         */
        ZERO,

        /**
         * attempt 1 skystone
         */
        ONE,

        /**
         * attempt 2 skystones
         */
        TWO
    }

    /**
     * number of stones to attempt to score during autonomous
     */
    private enum Stones{
        /**
         * attempt 0 extra stones
         */
        ZERO,

        /**
         * attempt 1 extra stone
         */
        ONE,

        /**
         * attempt 2 extra stones
         */
        TWO,

        /**
         * attempt 3 extra stones
         */
        THREE,

        /**
         * attempt 4 extra stones
         */
        FOUR,

        /**
         * attempt 5 extra stones
         */
        FIVE,

        /**
         * attempt 6 extra stones
         */
        SIX
    }

    /**
     * parking location and method during autonomous
     */
    private enum Park{
        /**
         * park near the wall with our robot
         */
        WALL,

        /**
         * park near the wall using tape measure
         */
        WALL_YEET,

        /**
         * park near the bridge using tape measure
         */
        BRIDGE_YEET,

        /**
         * park near the bridge with robot
         */
        BRIDGE,

        /**
         * don't park
         */
        NONE
    }

    /**
     * whether or not to attempt to score the foundation during autonomous
     */
    private enum Foundation{
        /**
         * attempt to score the foundation
         */
        YES,

        /**
         * don't attempt to score the foundation
         */
        NO
    }

    @Override
    public void runOpMode(){
        //initialize hardware members
        robot.init(hardwareMap);

        //initial values of all menu items
        alliance = Alliance.RED;
        delay = 0;
        startpos = StartingPos.FOUNDATION;
        skystones = Skystones.ONE;
        skystone = Skystone.MIDDLE;
        stones = Stones.ZERO;
        park = Park.WALL;
        foundation = Foundation.YES;

        dashboard = HalDashboard.createInstance(telemetry);
        doMenus();

        //if we are attempting a skystone, initiate the webcam
        if(skystones != Skystones.ZERO) {
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

            //different opencv positions to look based on starting position
            if(startpos == StartingPos.STONE1) {
                detector = new SSDetector(true);
            }
            else if(startpos == StartingPos.STONE2) {
                detector = new SSDetector(false);
            }

            webcam.openCameraDevice();
            webcam.setPipeline(detector);
            webcam.startStreaming(1920, 1080, OpenCvCameraRotation.UPRIGHT);

            //check for the position of the skystone
            while (!isStarted()) {
                skystone = detector.skystone;
                telemetry.addData("skystone:", skystone);
                telemetry.update();
            }
        }

        waitForStart();

        //TODO: MAKE ALL CASES AND TRANSFER OVER OTHER AUTONOMOUS PROGRAMS
        switch(alliance){
            case RED:{
                switch(startpos){
                    case STONE1:{
                        switch(skystones){
                            case ZERO:{

                                break;
                            }
                            case ONE:{

                                break;
                            }

                            case TWO:{

                                break;
                            }
                        }

                        break;
                    }

                    case STONE2:{
                        switch(skystones){
                            case ZERO:{

                                break;
                            }
                            case ONE:{

                                break;
                            }

                            case TWO:{

                                break;
                            }
                        }
                        break;
                    }

                    case FOUNDATION:{
                        switch(foundation){
                            case YES:{
                                encoderDrive(0.5,-35,-35, 0);
                                robot.gripFoundation();
                                sleep(1000);

                                turnRightCurvy(85,0.15);
                                encoderDrive(0.7,-30,-30, 0);
                                robot.releaseFoundation();
                                sleep(500);

                                switch(park){
                                    case WALL:{
                                        encStrafe(0.5, -9, 0);
                                        encoderDrive(0.7,55,55, 0);
                                        break;
                                    }

                                    case WALL_YEET:{

                                        break;
                                    }
                                }

                                break;
                            }

                            case NO:{

                                break;
                            }

                        }
                        break;
                    }
                }
                break;
            }

            case BLUE:{

                break;
            }
        }

    }

    //TODO: TEST ROADRUNNER
    //TODO: MAKE METHODS OF AUTONOMOUS ROUTINES LIKE scoreRedFoundation

    /**
     * Moves the robot a desired distance based on input.
     * Can also run intake during movement.
     *
     * @param speed       the desired speed to move at
     * @param leftInches  distance to move the left wheels
     * @param rightInches distance to move the right wheels
     * @param direction   direction to move the intake in (- is in, + is out)
     */
    private void encoderDrive(double speed, double leftInches, double rightInches, double direction){
        int LEFT_TARGET, RIGHT_TARGET;
        if(opModeIsActive()){
            LEFT_TARGET = (int)(leftInches*robot.getCPI()) + robot.frontLeft.getCurrentPosition();
            RIGHT_TARGET = (int)(rightInches*robot.getCPI()) + robot.frontRight.getCurrentPosition();

            robot.driveSetTarget(LEFT_TARGET,RIGHT_TARGET);
            robot.driveSetMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.driveSetPowerAll(Math.abs(speed));
            while(robot.driveIsBusy() && opModeIsActive()){
                robot.intakeSetPower(direction);
            }

            robot.stopMotors();
            robot.driveSetMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     * Turn left and stop at a desired inputted angle. Uses the REV Expansion Hub's
     * built in IMU to turn.
     *
     * @param TARGET_ANGLE desired angle to turn to
     * @param power        desired power to turn at
     */
    private void turnLeft(final float TARGET_ANGLE, double power){
        while(opModeIsActive()){
            float currentAngle = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            while(currentAngle<=TARGET_ANGLE){
                currentAngle = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                robot.driveSetPower(power, -power, power, -power);
                telemetry.addData("Heading:",currentAngle);
                telemetry.update();
            }
            robot.stopMotors();
            break;
        }
    }

    /**
     * Creates an arc moving left of the robot. Uses the REV Expansion Hub's
     * built in IMU.
     *
     * @param TARGET_ANGLE desired angle to turn to
     * @param power        desired power to turn at
     */
    private void turnLeftCurvy(final float TARGET_ANGLE, double power){
        while(opModeIsActive()){
            float currentAngle = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            while(currentAngle>=-TARGET_ANGLE){
                currentAngle = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                robot.driveSetPower(power*4, power, power*4, power);
            }
            robot.stopMotors();
            break;
        }
    }

    /**
     * Turn right and stop at a desired inputted angle. Uses the REV Expansion Hub's
     * built in IMU to turn.
     *
     * @param TARGET_ANGLE desired angle to turn to
     * @param power        desired power to turn at
     */
    private void turnRight(final float TARGET_ANGLE, double power){
        while(opModeIsActive()){
            float currentAngle = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            while(currentAngle>=-TARGET_ANGLE){
                currentAngle = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                robot.driveSetPower(-power, power, -power, power);
            }
            robot.stopMotors();
            break;
        }
    }

    /**
     * Creates an arc moving right of the robot. Uses the REV Expansion Hub's
     * built in IMU.
     *
     * @param TARGET_ANGLE desired angle to turn to
     * @param power        desired power to turn at
     */
    private void turnRightCurvy(final float TARGET_ANGLE, double power){
        while(opModeIsActive()){
            float currentAngle = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            while(currentAngle>=-TARGET_ANGLE){
                currentAngle = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                robot.driveSetPower(power, power*6, power, power*6);
            }
            robot.stopMotors();
            break;
        }
    }

    /**
     * Drive based on an inputted distance and speed. Uses the REV IMU
     * to attempt to drive straight (stay on its current heading, similar to
     * a PID Drive).
     *
     * @param speed     desired speed to move at
     * @param distance  desired distance to move
     * @param angle     angle to stay at
     * @param direction power to run intake at
     */
    private void encImuDrive(double speed, double distance, double angle, double direction){
        int LEFT_TARGET, RIGHT_TARGET;
        double error;
        double steer;
        double leftSpeed, rightSpeed;
        double max;

        if(opModeIsActive()){
            LEFT_TARGET = (int)(distance*robot.getCPI()) + robot.frontLeft.getCurrentPosition();
            RIGHT_TARGET = (int)(distance*robot.getCPI()) + robot.frontRight.getCurrentPosition();

            robot.driveSetTarget(LEFT_TARGET,RIGHT_TARGET);
            robot.driveSetMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.driveSetPowerAll(Math.abs(speed));

            while(robot.driveIsBusy() && opModeIsActive()){
                error = robot.getError(angle);
                steer = robot.getSteer(error, robot.P_DRIVE_COEFF);

                if (distance < 0) {
                    steer *= -1.0;
                }

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));

                if(max > 1.0){
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                robot.intakeSetPower(direction);
                robot.driveSetPower(leftSpeed, rightSpeed, leftSpeed, rightSpeed);

            }

            robot.stopMotors();
            robot.driveSetMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     * Strafe specific distances based on input.
     * - is strafe left, + is strafe right
     * Can also run the intake.
     * @param speed     speed to move at
     * @param distance  distance to move
     * @param direction power to move intake at
     */
    private void encStrafe(double speed, double distance, double direction){
        int flTarget, frTarget, blTarget, brTarget;

        if(opModeIsActive()){
            flTarget = (robot.frontLeft.getCurrentPosition()) + (int)(distance*robot.getCPI());
            frTarget = (robot.frontRight.getCurrentPosition()) - (int)(distance*robot.getCPI());
            blTarget = (robot.backLeft.getCurrentPosition()) - (int)(distance*robot.getCPI());
            brTarget = (robot.backRight.getCurrentPosition()) + (int)(distance*robot.getCPI());

            robot.driveSetTargetInd(flTarget, frTarget, blTarget, brTarget);
            robot.driveSetMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.driveSetPowerAll(Math.abs(speed));

            while(robot.driveIsBusy() && opModeIsActive()){
                robot.intakeSetPower(direction);
            }

            robot.stopMotors();
            robot.driveSetMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     * Strafe diagonally using only the front left and back right motors.
     * Can also run the intake.
     *
     * @param speed     speed to move at
     * @param distance  distance to move at
     * @param direction direction to intake (- for in + for out)
     */
    private void encStrafeFlBr(double speed, double distance, double direction){
        int flTarget, brTarget;

        if(opModeIsActive()){
            flTarget = (robot.frontLeft.getCurrentPosition()) + (int)(distance*robot.getCPI());
            brTarget = (robot.backRight.getCurrentPosition()) + (int)(distance*robot.getCPI());

            robot.frontLeft.setTargetPosition(flTarget);
            robot.backRight.setTargetPosition(brTarget);

            robot.driveSetMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.driveSetPower(Math.abs(speed), 0, 0 ,Math.abs(speed));

            while(robot.driveIsBusy() && opModeIsActive()){
                robot.intakeSetPower(direction);
            }

            robot.stopMotors();
            robot.driveSetMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     * Strafe diagonally using only the front right and back left motors.
     * Can also run the intake.
     *
     * @param speed     speed to move at
     * @param distance  distance to move at
     * @param direction direction to intake (- for in + for out)
     */
    private void encStrafeFrBl(double speed, double distance, double direction){
        int frTarget, blTarget;

        if(opModeIsActive()){
            frTarget = (robot.frontLeft.getCurrentPosition()) + (int)(distance*robot.getCPI());
            blTarget = (robot.backRight.getCurrentPosition()) + (int)(distance*robot.getCPI());

            robot.frontRight.setTargetPosition(frTarget);
            robot.backLeft.setTargetPosition(blTarget);

            robot.driveSetMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.driveSetPower(0, Math.abs(speed), Math.abs(speed), 0);

            while(robot.driveIsBusy() && opModeIsActive()){
                robot.intakeSetPower(direction);
            }

            robot.stopMotors();
            robot.driveSetMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     * park using the tape measure parking mechanism
     */
    private void yeetPark(){
        robot.yeet1.setPower(0.5);
        robot.yeet2.setPower(0.5);
        sleep(1250);
        robot.stopMotors();
    }

    //-------------------------------------- MENU -----------------------------------------------

    @Override
    public boolean isMenuUpButton() { return gamepad1.dpad_up; }

    @Override
    public boolean isMenuDownButton() { return gamepad1.dpad_down; }

    @Override
    public boolean isMenuEnterButton() { return gamepad1.dpad_right; }

    @Override
    public boolean isMenuBackButton() { return gamepad1.dpad_left; }

    private void doMenus(){
        FtcChoiceMenu<Alliance> allianceMenu = new FtcChoiceMenu<>("Alliance:", null, this);
        FtcChoiceMenu<StartingPos> startPosMenu = new FtcChoiceMenu<>("Start Position:", allianceMenu, this);
        FtcChoiceMenu<Skystones> skystonesMenu = new FtcChoiceMenu<>("Number of Skystones:", startPosMenu, this);
        FtcChoiceMenu<Stones> stonesMenu = new FtcChoiceMenu<>("Number of Stones:", skystonesMenu, this);
        FtcChoiceMenu<Foundation> foundationMenu = new FtcChoiceMenu<>("Move the Foundation:", stonesMenu, this);
        FtcChoiceMenu<Park> parkMenu = new FtcChoiceMenu<>("Park:", foundationMenu, this);
        FtcValueMenu delayMenu = new FtcValueMenu("Delay:", parkMenu, this, 0, 25000, 500, 0, "%.0f msec");

        allianceMenu.addChoice("Red", Alliance.RED, true, startPosMenu);
        allianceMenu.addChoice("Blue", Alliance.BLUE, false, startPosMenu);

        startPosMenu.addChoice("Stone Side Far", StartingPos.STONE1, false, skystonesMenu);
        startPosMenu.addChoice("Stone Side Close", StartingPos.STONE2, true, skystonesMenu);
        startPosMenu.addChoice("Foundation Side", StartingPos.FOUNDATION, false, skystonesMenu);

        skystonesMenu.addChoice("Zero", Skystones.ZERO, false, stonesMenu);
        skystonesMenu.addChoice("One", Skystones.ONE, true, stonesMenu);
        skystonesMenu.addChoice("Two", Skystones.TWO, false, stonesMenu);

        stonesMenu.addChoice("Zero", Stones.ZERO, true, foundationMenu);
        stonesMenu.addChoice("One", Stones.ONE, false, foundationMenu);
        stonesMenu.addChoice("Two", Stones.TWO, false, foundationMenu);
        stonesMenu.addChoice("Three", Stones.THREE, false, foundationMenu);
        stonesMenu.addChoice("Four", Stones.FOUR, false, foundationMenu);

        if(!(skystonesMenu.getCurrentChoiceObject() == Skystones.TWO)) {
            stonesMenu.addChoice("Five", Stones.FIVE, false, foundationMenu);
        }

        if(skystonesMenu.getCurrentChoiceObject() == Skystones.ZERO) {
            stonesMenu.addChoice("Six", Stones.SIX, false, foundationMenu);
        }

        foundationMenu.addChoice("Yes", Foundation.YES, true, parkMenu);
        foundationMenu.addChoice("No", Foundation.NO, false, parkMenu);

        parkMenu.addChoice("Wall", Park.WALL, true, delayMenu);
        parkMenu.addChoice("Wall Yeet", Park.WALL_YEET, false, delayMenu);
        parkMenu.addChoice("Bridge", Park.BRIDGE, false, delayMenu);
        parkMenu.addChoice("Bridge Yeet", Park.BRIDGE_YEET, false, delayMenu);
        parkMenu.addChoice("No Park", Park.NONE, false, delayMenu);

        delayMenu.setChildMenu(null);

        FtcMenu.walkMenuTree(allianceMenu, this);
        alliance = allianceMenu.getCurrentChoiceObject();
        startpos = startPosMenu.getCurrentChoiceObject();
        skystones = skystonesMenu.getCurrentChoiceObject();
        stones = stonesMenu.getCurrentChoiceObject();
        foundation = foundationMenu.getCurrentChoiceObject();
        park = parkMenu.getCurrentChoiceObject();
        delay = (int) delayMenu.getCurrentValue();

        dashboard.displayPrintf(0, "Alliance: %s (%s)", allianceMenu.getCurrentChoiceText(), alliance.toString());
        dashboard.displayPrintf(1, "Start Position: %s (%s)", startPosMenu.getCurrentChoiceText(), startpos.toString());
        dashboard.displayPrintf(2, "Skystones: %s (%s)", skystonesMenu.getCurrentChoiceText(), skystones.toString());
        dashboard.displayPrintf(3, "Stones: %s (%s)", stonesMenu.getCurrentChoiceText(), stones.toString());
        dashboard.displayPrintf(4, "Foundation: %s (%s)", foundationMenu.getCurrentChoiceText(), foundation.toString());
        dashboard.displayPrintf(5, "Park: %s (%s)", parkMenu.getCurrentChoiceText(), park.toString());
        dashboard.displayPrintf(6, "Delay: %d msec", delay);
    }
    //-------------------------------------- END MENU --------------------------------------------



    //-------------------------------------- DETECTOR --------------------------------------------

    static class SSDetector extends OpenCvPipeline {
        private Mat workingMatrix = new Mat();
        Skystone skystone = Skystone.RIGHT;
        boolean wall; //whether or not the robot is starting along the wall

        SSDetector (boolean wall){
            this.wall = wall;
        }

        @Override
        public final Mat processFrame(Mat input) {
            Mat matLeft, matCenter, matRight;
            input.copyTo(workingMatrix);

            if (workingMatrix.empty()) {
                return input;
            }

            Imgproc.cvtColor(workingMatrix, workingMatrix, Imgproc.COLOR_RGB2YCrCb);

            //use different matrices based on starting position of robot
            if (wall) {
                matLeft = workingMatrix.submat(710, 810, 600, 760);
                matCenter = workingMatrix.submat(710, 810, 960, 1120);
                matRight = workingMatrix.submat(710, 810, 1320, 1480);

                Imgproc.rectangle(workingMatrix, new Rect(600, 710, 160, 100), new Scalar(0, 0, 255));
                Imgproc.rectangle(workingMatrix, new Rect(960, 710, 160, 100), new Scalar(0, 0, 255));
                Imgproc.rectangle(workingMatrix, new Rect(1320, 710, 160, 100), new Scalar(0, 0, 255));
            }
            else {
                matLeft = workingMatrix.submat(650, 750, 550, 710);
                matCenter = workingMatrix.submat(650, 750, 910, 1070);
                matRight = workingMatrix.submat(650, 750, 1270, 1430);

                Imgproc.rectangle(workingMatrix, new Rect(550, 650, 160, 100), new Scalar(0, 0, 255));
                Imgproc.rectangle(workingMatrix, new Rect(910, 650, 160, 100), new Scalar(0, 0, 255));
                Imgproc.rectangle(workingMatrix, new Rect(1270, 650, 160, 100), new Scalar(0, 0, 255));
            }

            double leftSum = Core.sumElems(matLeft).val[2];
            double centerSum = Core.sumElems(matCenter).val[2];
            double rightSum = Core.sumElems(matRight).val[2];

            //use deductive reasoning to determine the position of the skystone
            if (leftSum > centerSum) {
                if (leftSum > rightSum) {
                    skystone = Skystone.LEFT;
                }
                else {
                    skystone = Skystone.RIGHT;
                }
            }
            else {
                if (centerSum > rightSum) {
                    skystone = Skystone.MIDDLE;
                }
                else {
                    skystone = Skystone.RIGHT;
                }
            }

            return workingMatrix;
        }
    }
    //-------------------------------------- END DETECTOR -----------------------------------------
}
