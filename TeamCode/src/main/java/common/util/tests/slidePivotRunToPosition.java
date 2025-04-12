package common.util.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import common.robot.sensors.LimitSwitch;

@TeleOp
@Disabled
public class slidePivotRunToPosition extends OpMode{

    DcMotorEx spl;
    DcMotorEx spr;
    LimitSwitch slidePivotLeftLimit, slidePivotRightLimit;

    @Override
    public void init() {
        spl = hardwareMap.get(DcMotorEx.class, "slidepivotLeft");
        spr = hardwareMap.get(DcMotorEx.class, "slidepivotRight");

        slidePivotLeftLimit = new LimitSwitch(hardwareMap.get(DigitalChannel.class, "hpivotleftLimit"));
        slidePivotRightLimit = new LimitSwitch(hardwareMap.get(DigitalChannel.class, "hpivotrightLimit"));

        spl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        spr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }

    @Override
    public void loop() {
        if (gamepad1.dpad_up){
//            spr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            spl.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            spr.setPower(1);
            spl.setPower(1);
//            spr.setTargetPosition(1040);
            spl.setTargetPosition(1040);

        }


        else if (gamepad1.dpad_down){
//            spr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            spl.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            spl.setPower(-1);
            spr.setPower(-1);
//            spr.setTargetPosition(0);
            spl.setTargetPosition(0);
            if ((spl.getCurrentPosition()<5)||slidePivotLeftLimit.isPressed()||slidePivotRightLimit.isPressed()){
                spr.setPower(0);
                spl.setPower(0);
            }
            sleep(100);
        }

        telemetry.addData("spr", spr.getCurrentPosition());
        telemetry.addData("spl", spl.getCurrentPosition());
        telemetry.addData("spl power consumption", spl.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("spr power consumption", spr.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("spr limit pressed", slidePivotRightLimit.isPressed());
        telemetry.addData("spl limit pressed", slidePivotLeftLimit.isPressed());

    }
    private void sleep(int millis) {
        try {
            Thread.sleep(millis);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}