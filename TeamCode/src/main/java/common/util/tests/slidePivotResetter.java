package common.util.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class slidePivotResetter extends OpMode {

    private DcMotorEx slidePivotLeft;
    private DcMotorEx slidePivotRight;

    @Override
    public void init() {
        slidePivotLeft = hardwareMap.get(DcMotorEx.class, "slidepivotLeft");
        slidePivotRight = hardwareMap.get(DcMotorEx.class, "slidepivotRight");
    }

    @Override
    public void loop() {
        if (gamepad1.dpad_up){
            slidePivotLeft.setPower(0.4);
            slidePivotRight.setPower(0.4);
            sleep(75);
            slidePivotLeft.setPower(0);
            slidePivotRight.setPower(0);
            sleep(200);
        }
        if (!gamepad1.dpad_up){
            slidePivotLeft.setPower(0);
            slidePivotLeft.setPower(0);
        }
        if (gamepad1.dpad_down){
            slidePivotLeft.setPower(-0.4);
            slidePivotRight.setPower(-0.4);
            sleep(75);
            slidePivotLeft.setPower(0);
            slidePivotRight.setPower(0);
            sleep(200);
        }
        if (!gamepad1.dpad_down){
            slidePivotLeft.setPower(0);
            slidePivotLeft.setPower(0);
        }
    }

    private void sleep(long millis) {
        try {
            Thread.sleep(millis);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}
