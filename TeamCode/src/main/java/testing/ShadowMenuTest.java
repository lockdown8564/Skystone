package testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import org.firstinspires.ftc.robotcore.external.ClassFactory;

import ftclib.FtcChoiceMenu;
import ftclib.FtcMenu;
import ftclib.FtcValueMenu;
import hallib.HalDashboard;

/**
 * testing menus for shadow
 * @see MenuTest
 * @see ftclib.FtcChoiceMenu
 * @see ftclib.FtcValueMenu
 * made: 1/2/20
 * last updated: 2/2/20
 */

@Autonomous(name = "shadow menu test", group = "test")
public class ShadowMenuTest extends LinearOpMode implements FtcMenu.MenuButtons{
    private ShadowTestHardware robot = new ShadowTestHardware();

    private enum Alliance{
        BLUE,
        RED
    }

    /**
     * starting position of robot in autonomous
     */
    private enum StartingPos{
        /**
         * foundation side along the side wall
         */
        FOUNDATION1,

        /**
         * foundation side one tile away from the side wall
         */
        FOUNDATION2,

        /**
         * stone side along the side wall
         */
        STONE1,

        /**
         * stone side one tile away from the side wall
         */
        STONE2
    }

    private enum Skystone{
        LEFT,
        MIDDLE,
        RIGHT
    }

    private enum Skystones{
        ZERO,
        ONE,
        TWO
    }

    private enum Stones{
        ZERO,
        ONE,
        TWO,
        THREE,
        FOUR,
        FIVE,
        SIX
    }

    private enum Park{
        WALL,
        BRIDGE,
        NONE
    }

    private enum Foundation{
        YES,
        NO
    }

    Alliance alliance = Alliance.RED;
    int delay = 0;
    StartingPos startpos = StartingPos.STONE1;
    Skystones skystones = Skystones.ONE;
    Skystone skystone = Skystone.LEFT;
    Stones stones = Stones.ZERO;
    Park park = Park.WALL;
    Foundation foundation = Foundation.YES;

    private static HalDashboard dashboard;

    @Override
    public void runOpMode() {
        dashboard = HalDashboard.createInstance(telemetry);
        doMenus();

        waitForStart();

        switch(alliance){
            case RED:{
                switch(startpos){
                    case STONE1:{

                        break;
                    }

                    case STONE2:{

                        break;
                    }

                    case FOUNDATION1:{

                        break;
                    }

                    case FOUNDATION2:{

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

    private void encoderDrive(double speed, double leftInches, double rightInches){
        int LEFT_TARGET, RIGHT_TARGET;
        if(opModeIsActive()){
            LEFT_TARGET = (int)(leftInches*robot.getCPI()) + robot.frontLeft.getCurrentPosition();
            RIGHT_TARGET = (int)(rightInches*robot.getCPI()) + robot.frontRight.getCurrentPosition();

            robot.driveSetTarget(LEFT_TARGET,RIGHT_TARGET);
            robot.driveSetMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.driveSetPowerAll(Math.abs(speed));
            while(robot.driveIsBusy() && opModeIsActive()){
            }

            robot.stopMotors();
            robot.driveSetMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    private void encoderDriveIntake(double speed, double leftInches, double rightInches, double direction){
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

    private void turnRightCurvy(final float TARGET_ANGLE, double power){
        while(opModeIsActive()){
            float currentAngle = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            while(currentAngle>=-TARGET_ANGLE){
                currentAngle = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                robot.driveSetPower(power, power*4, power, power*4);
            }
            robot.stopMotors();
            break;
        }
    }

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

    private void turnLeftPivot(final float TARGET_ANGLE, double power){
        while(opModeIsActive()){
            float currentAngle = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            while(currentAngle<=TARGET_ANGLE){
                currentAngle = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                robot.driveSetPower(power, 0, power, 0);
                telemetry.addData("Heading:",currentAngle);
                telemetry.update();
            }
            robot.stopMotors();
            break;
        }
    }

    private void turnRightPivot(final float TARGET_ANGLE, double power){
        while(opModeIsActive()){
            float currentAngle = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            while(currentAngle>=-TARGET_ANGLE){
                currentAngle = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                robot.driveSetPower(0, power, 0, power);
            }
            robot.stopMotors();
            break;
        }
    }

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

    private void encImuDrive(double speed, double distance, double angle){
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

                robot.driveSetPower(leftSpeed, rightSpeed, leftSpeed, rightSpeed);

            }

            robot.stopMotors();
            robot.driveSetMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    // - is right, + is left
    private void imuStrafe(double speed, double distance, double angle){
        int flTarget, frTarget, blTarget, brTarget;
        double error;
        double steer;
        double leftSpeed, rightSpeed;
        double max;

        if(opModeIsActive()){
            flTarget = (robot.frontLeft.getCurrentPosition()) + (int)(distance*robot.getCPI());
            frTarget = (robot.frontRight.getCurrentPosition()) - (int)(distance*robot.getCPI());
            blTarget = (robot.backLeft.getCurrentPosition()) - (int)(distance*robot.getCPI());
            brTarget = (robot.backRight.getCurrentPosition()) + (int)(distance*robot.getCPI());

            robot.driveSetTargetInd(flTarget, frTarget, blTarget, brTarget);
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

                robot.driveSetPower(leftSpeed, rightSpeed, leftSpeed, rightSpeed);

            }

            robot.stopMotors();
            robot.driveSetMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
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
        FtcValueMenu delayMenu = new FtcValueMenu("Delay:", parkMenu, this, 0, 20000, 1000, 0, "%.0f msec");

        allianceMenu.addChoice("Red", Alliance.RED, true, startPosMenu);
        allianceMenu.addChoice("Blue", Alliance.BLUE, false, startPosMenu);

        startPosMenu.addChoice("Stone Side Far", StartingPos.STONE1, true, skystonesMenu);
        startPosMenu.addChoice("Stone Side Close", StartingPos.STONE2, false, skystonesMenu);
        startPosMenu.addChoice("Foundation Side Far", StartingPos.FOUNDATION1, false, skystonesMenu);
        startPosMenu.addChoice("Foundation Side Close", StartingPos.FOUNDATION2, false, skystonesMenu);

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
        parkMenu.addChoice("Bridge", Park.BRIDGE, false, delayMenu);
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

    public static HalDashboard getDashboard() {
        return dashboard;
    }
}
