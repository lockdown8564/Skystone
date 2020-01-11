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
 * testing menus for currently unnamed robot
 * @see MenuTest
 * @see ftclib.FtcChoiceMenu
 * @see ftclib.FtcValueMenu
 * made: 1/2/20
 * last updated: 1/2/20
 */

@Autonomous(name = "shadow menu test", group = "test")
public class ShadowMenuTest extends LinearOpMode implements FtcMenu.MenuButtons{
    private enum Alliance{
        BLUE,
        RED
    }
    private enum StartingSide{
        FOUNDATION,
        STONES
    }
    private enum Skystones{
        ZERO,
        ONE,
        TWO
    }
    @Override
    public void runOpMode() {

    }

    @Override
    public boolean isMenuUpButton() { return gamepad1.dpad_up; }

    @Override
    public boolean isMenuDownButton() { return gamepad1.dpad_down; }

    @Override
    public boolean isMenuEnterButton() { return gamepad1.dpad_right; }

    @Override
    public boolean isMenuBackButton() { return gamepad1.dpad_left; }

    private void doMenus(){

    }

}
