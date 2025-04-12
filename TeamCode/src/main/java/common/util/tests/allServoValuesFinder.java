package common.util.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Servo Test", group = "TeleOp")
@Disabled
public class allServoValuesFinder extends OpMode {

    private Servo armClawServo, armWristServo, armPitchServo, armPivotServo, ledFlashServo;
    private double pos1 = 0.5, pos2 = 0.5, pos3 = 0.5, pos4 = 0.5, pos5=0.5;
    private final double INCREMENT = 0.01; // Step size for position changes

    @Override
    public void init() {
        armClawServo = hardwareMap.get(Servo.class, "armclawServo");
        armPivotServo = hardwareMap.get(Servo.class, "armpivotServo");
        armWristServo = hardwareMap.get(Servo.class, "armwristServo");
        armPitchServo = hardwareMap.get(Servo.class, "armpitchServo");
        ledFlashServo = hardwareMap.get(Servo.class, "ledflashServo");

        armClawServo.setPosition(pos1);
        armPivotServo.setPosition(pos2);
        armWristServo.setPosition(pos3);
        armPitchServo.setPosition(pos4);
        ledFlashServo.setPosition(pos5);


    }
    @Override
    public void loop(){
            // Increase servo positions
            if (gamepad1.a) pos1 = Math.min(1.0, pos1 + INCREMENT);
            if (gamepad1.b) pos2 = Math.min(1.0, pos2 + INCREMENT);
            if (gamepad1.x) pos3 = Math.min(1.0, pos3 + INCREMENT);
            if (gamepad1.y) pos4 = Math.min(1.0, pos4 + INCREMENT);
            if (gamepad1.dpad_up) pos5 = Math.min(1.0, pos5 + INCREMENT);


            // Decrease servo positions
            if (gamepad1.dpad_down) pos1 = Math.max(0.0, pos1 - INCREMENT);
            if (gamepad1.dpad_left) pos2 = Math.max(0.0, pos2 - INCREMENT);
            if (gamepad1.dpad_right) pos3 = Math.max(0.0, pos3 - INCREMENT);
            if (gamepad1.left_bumper) pos4 = Math.max(0.0, pos4 - INCREMENT);
            if (gamepad1.right_bumper) pos5 = Math.min(1.0, pos5 - INCREMENT);


            // Set servo positions
            armClawServo.setPosition(pos1);
            armPivotServo.setPosition(pos2);
            armWristServo.setPosition(pos3);
            armPitchServo.setPosition(pos4);
            ledFlashServo.setPosition(pos5);
            sleep(50);


            // Telemetry feedback
            telemetry.addData("armClawServo Pos", pos1);
            telemetry.addData("armPivotServo Pos", pos2);
            telemetry.addData("armWristServo Pos", pos3);
            telemetry.addData("armPitchServo Pos", pos4);
        telemetry.addData("led Pos", pos5);
            telemetry.update();

        }
    private void sleep(long millis) {
        try {
            Thread.sleep(millis);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    }




