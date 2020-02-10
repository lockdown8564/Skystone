package testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.Range;

/**
 * testing throughbore encoder teleop
 * created: 2/2/20
 * last updated: 2/2/20
 */

@Disabled
@TeleOp(name="throughbore test",group="test")
public class ThroughboreTeleTest extends OpMode {
    private ShadowTestHardware robot = new ShadowTestHardware();

    @Override
    public void init(){
        robot.init(hardwareMap);
    }

    @Override
    public void init_loop(){}
    @Override
    public void start(){}
    @Override
    public void loop(){
        if (gamepad1.left_stick_y != 0) {
            robot.slide.setPower(gamepad1.left_stick_y * .5);
        }

        else if(gamepad1.x){
            robot.swing.setTargetPosition(50);
            robot.swing.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.swing.setPower(0.5);
            while(robot.swing.isBusy()){
            }

            robot.stopMotors();
            robot.driveSetMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        else if(gamepad1.y){
            robot.swing.setTargetPosition(-50);
            robot.swing.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.swing.setPower(0.5);
            while(robot.swing.isBusy()){
            }

            robot.stopMotors();
            robot.driveSetMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        else if(gamepad1.a){
            robot.swing.setTargetPosition(50);
            robot.swing.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.swing.setPower(-0.5);
            while(robot.swing.isBusy()){
            }

            robot.stopMotors();
            robot.driveSetMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        else if(gamepad1.b){
            robot.swing.setTargetPosition(-50);
            robot.swing.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.swing.setPower(-0.5);
            while(robot.swing.isBusy()){
            }

            robot.stopMotors();
            robot.driveSetMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        else {
            robot.slide.setPower(0);
        }

        telemetry.addData("Swing:",robot.swing.getCurrentPosition());
        telemetry.update();

    }

    @Override
    public void stop(){
        robot.stopMotors();
    }
}
