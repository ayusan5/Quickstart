package common.util.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp
@Disabled
public class armclawfindersigma extends OpMode {

    private Servo armclawServo;

    @Override
    public void init() {
        armclawServo = hardwareMap.get(Servo.class, "armclawServo");

        armclawServo.setPosition(0.5);
    }

    @Override
    public void loop() {
        if (gamepad1.dpad_down){
            armclawServo.setPosition(armclawServo.getPosition()-0.01);
            sleep(40);
        }
        if (gamepad1.dpad_up){
            armclawServo.setPosition(armclawServo.getPosition()+0.01);
            sleep(40);
        }
    }
    private void sleep(int millis) {
        try {
            Thread.sleep(millis);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}
