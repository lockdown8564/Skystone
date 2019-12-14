package testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

/**
 * testing skystone autonomous
 * created 12/13/19
 */
@Disabled
@Autonomous(name = "red stone park", group = "test")
public class SonicRedStoneAuto extends LinearOpMode {
    private SonicTestHardware robot = new SonicTestHardware();
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";
    private static final String VUFORIA_KEY = "AbxdFCj/////AAABmWeoUc73KkCmj5FqhZSgVY+F7nUU10SIb4BdCXOy/K6S02LySDpnqQLBeJJpbV4/TLkXnY87P/aoBefrJSvjPOt4zbx8v9JQrqvJ5ZWDRqxmtjzjSJz1o9XW5BZ9AlOBeBiFc5MaQLCHXajJorWs+hVDGoXLTDGHSbBiZx1zQKyCI5P5geu7TI3dllw3nE7pECZOXRRaYHUG2snAbJyOk5p10jk8jy7cv3QahxWECRKkrRGjZ5MTb6gfQRkheZVHEE+May9jY2lUoe4u6KCvXElFlDSdvNgzd0f3IGgSOjzPyJkBGaqO7fg1CrKAh6a7iyFL1Ktkhe25ZEVsIrGNWHiHIW2EC12bf00Vk2QcwWNQ";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    private Skystone skystone = Skystone.RIGHT;

    private enum Skystone{
        LEFT,
        MIDDLE,
        RIGHT
    }

    @Override
    public void runOpMode(){
        robot.init(hardwareMap);
        robot.driveSetMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        }
        else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        if (tfod != null) {
            tfod.activate();
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        waitForStart();

        //releaseIntake();
        encoderDrive(0.7,-10,-10);

        if (tfod != null){
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null){
                for(Recognition recognition : updatedRecognitions){
                    if(recognition.getLabel().equals("Skystone")){
                        if(recognition.getLeft() < 100){
                            skystone = Skystone.LEFT;
                        }
                        else if(recognition.getLeft() > 100 && recognition.getLeft() < 350){
                            skystone = Skystone.MIDDLE;
                        }
                        else{
                            skystone = Skystone.RIGHT;
                        }
                        telemetry.addData("Skystone:",recognition.getLabel());
                        telemetry.update();
                    }
                }
            }
        }

        sleep(1000);
        turnLeft(90,0.4);

        switch(skystone){
            case LEFT:{

                break;
            }
            case MIDDLE:{

                break;
            }
            case RIGHT:{

                break;
            }
        }

    }

    private void encoderDrive(double speed, double leftInches, double rightInches){
        int LEFT_TARGET, RIGHT_TARGET;
        if(opModeIsActive()){
            LEFT_TARGET = (int)(leftInches*robot.COUNTS_PER_INCH) + robot.frontLeft.getCurrentPosition();
            RIGHT_TARGET = (int)(rightInches*robot.COUNTS_PER_INCH) + robot.frontRight.getCurrentPosition();

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
            LEFT_TARGET = (int)(leftInches*robot.COUNTS_PER_INCH) + robot.frontLeft.getCurrentPosition();
            RIGHT_TARGET = (int)(rightInches*robot.COUNTS_PER_INCH) + robot.frontRight.getCurrentPosition();

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

    private void turnRight(final float TARGET_ANGLE, double power){
        while(opModeIsActive()){
            float currentAngle = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            if(currentAngle>=-TARGET_ANGLE){
                robot.driveSetPower(-power,power);
            }
            else{
                robot.stopMotors();
                break;
            }
        }
    }

    private void turnLeft(final float TARGET_ANGLE, double power){
        while(opModeIsActive()){
            float currentAngle = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            if(currentAngle<=TARGET_ANGLE){
                robot.driveSetPower(power,-power);
            }
            else{
                robot.stopMotors();
                break;
            }
        }
    }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.4;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    private void releaseIntake(){
        robot.hook.setPosition(0.9);
        robot.latch.setPosition(0);
    }
    /*
    private void resetArm(int TARGET){
        if(opModeIsActive()){
            robot.arm.setTargetPosition(TARGET);
            robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.arm.setPower(0.5);
            while(robot.arm.isBusy() && opModeIsActive()){
            }

            robot.stopMotors();
            robot.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }*/

}
