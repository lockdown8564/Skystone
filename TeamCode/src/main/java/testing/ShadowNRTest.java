package testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Disabled
@Autonomous (name = "nr 40 test", group = "test")
public class ShadowNRTest extends LinearOpMode {
    @Override
    public void runOpMode(){
        DcMotor motor = hardwareMap.dcMotor.get("motor");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        motor.setTargetPosition(545);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(0.5);
        while(motor.isBusy()){

        }
        motor.setPower(0);

        telemetry.addData("Encoder:",motor.getCurrentPosition());
        telemetry.update();
    }
}
