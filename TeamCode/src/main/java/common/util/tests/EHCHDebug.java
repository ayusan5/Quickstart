package common.util.tests;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "12 Servo and 8 DC Motor Test", group = "TeleOp")
@Disabled
public class EHCHDebug extends OpMode {

    // Declare 8 DC Motors and 12 Servos
    private DcMotor motor0, motor1, motor2, motor3, motor4, motor5, motor6, motor7;
    private Servo servo0, servo1, servo2, servo3, servo4, servo5, servo6, servo7, servo8, servo9, servo10, servo11;
    private DigitalChannel dc0, dc1, dc2, dc3, dc4, dc5, dc6, dc7, dc8, dc9, dc10, dc11, dc12, dc13, dc14, dc15;
    private AnalogInput as0, as1, as2, as3, as4, as5, as6, as7;
    private BNO055IMU imu;
    private RevColorSensorV3 cs0, cs1, cs2;

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

        dc0 = hardwareMap.digitalChannel.get("dc0");
        dc1 = hardwareMap.digitalChannel.get("dc1");
        dc2 = hardwareMap.digitalChannel.get("dc2");
        dc3 = hardwareMap.digitalChannel.get("dc3");
        dc4 = hardwareMap.digitalChannel.get("dc4");
        dc5 = hardwareMap.digitalChannel.get("dc5");
        dc6 = hardwareMap.digitalChannel.get("dc6");
        dc7 = hardwareMap.digitalChannel.get("dc7");
        dc8 = hardwareMap.digitalChannel.get("dc8");
        dc9 = hardwareMap.digitalChannel.get("dc9");
        dc10 = hardwareMap.digitalChannel.get("d10");
        dc11= hardwareMap.digitalChannel.get("dc11");
        dc12= hardwareMap.digitalChannel.get("dc12");
        dc13= hardwareMap.digitalChannel.get("dc13");
        dc14= hardwareMap.digitalChannel.get("dc14");
        dc15= hardwareMap.digitalChannel.get("dc15");

        as1 = hardwareMap.analogInput.get("as1");
        as0 = hardwareMap.analogInput.get("as0");
        as2 = hardwareMap.analogInput.get("as2");
        as3 = hardwareMap.analogInput.get("as3");
        as4 = hardwareMap.analogInput.get("as4");
        as5 = hardwareMap.analogInput.get("as5");
        as6 = hardwareMap.analogInput.get("as6");
        as7 = hardwareMap.analogInput.get("as7");

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        cs0 = hardwareMap.get(RevColorSensorV3.class, "cs0");
        cs1 = hardwareMap.get(RevColorSensorV3.class, "cs1");
        cs2 = hardwareMap.get(RevColorSensorV3.class, "cs2");



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
        imu.initialize(new BNO055IMU.Parameters());
        telemetry.addData("IMU Status", imu.getAcceleration());

        telemetry.addData("cs0", cs0.getLightDetected());
        telemetry.addData("cs1", cs1.getLightDetected());
        telemetry.addData("cs2", cs2.getLightDetected());

        telemetry.addData("dc0", dc0.getState());
        telemetry.addData("dc1", dc1.getState());
        telemetry.addData("dc2", dc2.getState());
        telemetry.addData("dc3", dc3.getState());
        telemetry.addData("dc4", dc4.getState());
        telemetry.addData("dc5", dc5.getState());
        telemetry.addData("dc6", dc6.getState());
        telemetry.addData("dc7", dc7.getState());
        telemetry.addData("dc8", dc8.getState());
        telemetry.addData("dc9", dc9.getState());
        telemetry.addData("dc10", dc10.getState());
        telemetry.addData("dc11", dc11.getState());
        telemetry.addData("dc12", dc12.getState());
        telemetry.addData("dc13", dc13.getState());
        telemetry.addData("dc14", dc14.getState());
        telemetry.addData("dc15", dc15.getState());

        telemetry.addData("as0", as0.getVoltage());
        telemetry.addData("as1", as1.getVoltage());
        telemetry.addData("as2", as2.getVoltage());
        telemetry.addData("as3", as3.getVoltage());
        telemetry.addData("as4", as4.getVoltage());
        telemetry.addData("as5", as5.getVoltage());
        telemetry.addData("as6", as6.getVoltage());
        telemetry.addData("as7", as7.getVoltage());

        telemetry.update();


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