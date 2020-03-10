package ftc8564;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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
import testing.ShadowRedSSTest;

@Autonomous (name = "shadow auto", group = "ftc8564")
@Disabled
public class ShadowAutonomous extends LinearOpMode implements FtcMenu.MenuButtons {
    private ShadowHardware robot = new ShadowHardware();
    private OpenCvCamera webcam;
    private SSDetector detector;

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
        WALL_YEET,
        BRIDGE_YEET,
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
    public void runOpMode(){
        robot.init(hardwareMap);
        dashboard = HalDashboard.createInstance(telemetry);
        doMenus();

        if(skystones != Skystones.ZERO) {
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

            if(startpos == StartingPos.STONE1) {
                detector = new SSDetector(true);
            }
            else if(startpos == StartingPos.STONE2) {
                detector = new SSDetector(false);
            }

            webcam.openCameraDevice();
            webcam.setPipeline(detector);
            webcam.startStreaming(1920, 1080, OpenCvCameraRotation.UPRIGHT);

            while (!isStarted()) {
                skystone = detector.skystone;
                telemetry.addData("skystone:", skystone);
                telemetry.update();
            }
        }

        waitForStart();

    }

    private void yeetPark(){
        robot.yeet1.setPower(0.5);
        robot.yeet2.setPower(0.5);
        sleep(1250);
        robot.yeet1.setPower(0);
        robot.yeet2.setPower(0);
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

    public static HalDashboard getDashboard() {
        return dashboard;
    }

    //-------------------------------------- END MENU --------------------------------------------



    //-------------------------------------- DETECTOR --------------------------------------------

    static class SSDetector extends OpenCvPipeline {
        private Mat workingMatrix = new Mat();
        Skystone skystone = Skystone.RIGHT;
        boolean wall;

        public SSDetector(boolean wall){
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

            if (leftSum > centerSum) {
                if (leftSum > rightSum) {
                    //skystone is left
                    skystone = Skystone.LEFT;
                }
                else {
                    //skystone is right
                    skystone = Skystone.RIGHT;
                }
            }
            else {
                if (centerSum > rightSum) {
                    //skystone is center
                    skystone = Skystone.MIDDLE;
                }
                else {
                    //skystone is right
                    skystone = Skystone.RIGHT;
                }
            }

            return workingMatrix;
        }
    }
    //-------------------------------------- END DETECTOR -----------------------------------------
}
