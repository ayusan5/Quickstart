package common.util.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
@Disabled
public class DCServoDebug extends OpMode {

    // Declare 8 DC Motors and 12 Servos
    private DcMotor motor0, motor1, motor2, motor3, motor4, motor5, motor6, motor7;
    private Servo servo0, servo1, servo2, servo3, servo4, servo5, servo6, servo7, servo8, servo9, servo10, servo11;

    @Override
    public void init() {
        // Initialize DC Motors
        motor0 = hardwareMap.dcMotor.get("motor0");
        motor1 = hardwareMap.dcMotor.get("motor1");
        motor2 = hardwareMap.dcMotor.get("motor2");
        motor3 = hardwareMap.dcMotor.get("motor3");
        motor4 = hardwareMap.dcMotor.get("motor4");
        motor5 = hardwareMap.dcMotor.get("motor5");
        motor6 = hardwareMap.dcMotor.get("motor6");
        motor7 = hardwareMap.dcMotor.get("motor7");

        // Initialize 12 Servos
        servo0 = hardwareMap.servo.get("servo0");
        servo1 = hardwareMap.servo.get("servo1");
        servo2 = hardwareMap.servo.get("servo2");
        servo3 = hardwareMap.servo.get("servo3");
        servo4 = hardwareMap.servo.get("servo4");
        servo5 = hardwareMap.servo.get("servo5");
        servo6 = hardwareMap.servo.get("servo6");
        servo7 = hardwareMap.servo.get("servo7");
        servo8 = hardwareMap.servo.get("servo8");
        servo9 = hardwareMap.servo.get("servo9");
        servo10 = hardwareMap.servo.get("servo10");
        servo11 = hardwareMap.servo.get("servo11");

        // Set all servos to center position (0.5)
        for (Servo servo : new Servo[]{servo0, servo1, servo2, servo3, servo4, servo5, servo6, servo7, servo8, servo9, servo10, servo11}) {
            servo.setPosition(0.5);
        }
    }

    @Override
    public void loop() {
        // Dpad UP: Move all DC motors forward for 1 second
        if (gamepad1.dpad_up) {
            setMotorPower(1);
            sleep(1000); // Wait for 1 second
            stopMotors(); // Stop all motors after 1 second
        }

        // Dpad DOWN: Move all DC motors reverse for 1 second
        if (gamepad1.dpad_down) {
            setMotorPower(-1);
            sleep(1000); // Wait for 1 second
            stopMotors(); // Stop all motors after 1 second
        }

        // X Button: Move all servos to 0 (extreme left)
        if (gamepad1.x) {
            moveServos(0);
        }

        // Y Button: Move all servos to 1 (extreme right)
        if (gamepad1.y) {
            moveServos(1);
        }
    }

    // Helper method to set power to all DC motors
    private void setMotorPower(double power) {
        motor0.setPower(power);
        motor1.setPower(power);
        motor2.setPower(power);
        motor3.setPower(power);
        motor4.setPower(power);
        motor5.setPower(power);
        motor6.setPower(power);
        motor7.setPower(power);
    }

    // Helper method to stop all DC motors
    private void stopMotors() {
        setMotorPower(0);
    }

    // Helper method to move all servos to a specific position
    private void moveServos(double position) {
        servo0.setPosition(position);
        servo1.setPosition(position);
        servo2.setPosition(position);
        servo3.setPosition(position);
        servo4.setPosition(position);
        servo5.setPosition(position);
        servo6.setPosition(position);
        servo7.setPosition(position);
        servo8.setPosition(position);
        servo9.setPosition(position);
        servo10.setPosition(position);
        servo11.setPosition(position);
    }

    // Helper method to simulate sleep
    private void sleep(long millis) {
        try {
            Thread.sleep(millis);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}