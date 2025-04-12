package common.util.tests;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="servoControl 2.0 pmo" , group = "TeleOp")
@Disabled
public class servoControl extends OpMode {

    // Motor declarations
    private DcMotorEx slideLeft, slideRight;
    private DcMotorEx slidePivotLeft, slidePivotRight;

    // Servo declarations
    private Servo armclaw, armwrist, armpitch, armpivot, led;

    // PID components
    private PIDController slideController;
    private final double p = 0.01, i = 0, d = 0.0001, f = 0.003;
    private int slideTarget = 0;
    private boolean lastUp = false, lastDown = false;

    // Constants
    private static final int TICK_INCREMENT = 300;
    private static final int MAX_TICKS = 2050;
    private static final double MIN_POSITION = 0.0;
    private static final double MAX_POSITION = 1.0;

    @Override
    public void init() {
        initializeHardware();
        initializePID();
        setInitialPositions();
    }

    private void initializeHardware() {
        // Slide motors
        slideLeft = hardwareMap.get(DcMotorEx.class, "slideLeft");
        slideRight = hardwareMap.get(DcMotorEx.class, "slideRight");
        slideRight.setDirection(DcMotorEx.Direction.REVERSE);

        // Pivot motors
        slidePivotLeft = hardwareMap.get(DcMotorEx.class, "slidepivotLeft");
        slidePivotRight = hardwareMap.get(DcMotorEx.class, "slidepivotRight");

        // Servos
        armclaw = hardwareMap.get(Servo.class, "armclawServo");
        armwrist = hardwareMap.get(Servo.class, "armwristServo");
        armpitch = hardwareMap.get(Servo.class, "armpitchServo");
        armpivot = hardwareMap.get(Servo.class, "armpivotServo");
        led = hardwareMap.get(Servo.class, "ledflashServo");

        configureMotors();
    }

    private void configureMotors() {
        // Configure slide motors
        DcMotorEx[] slides = {slideLeft, slideRight};
        for (DcMotorEx motor : slides) {
            motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        }

        // Configure pivot motors
        DcMotorEx[] pivots = {slidePivotLeft, slidePivotRight};
        for (DcMotorEx motor : pivots) {
            motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        }
    }

    private void initializePID() {
        slideController = new PIDController(p, i, d);
        slideController.setTolerance(10);
    }

    private void setInitialPositions() {
        Servo[] servos = {armclaw, armwrist, armpitch, armpivot, led};
        for (Servo servo : servos) {
            servo.setPosition(0.5);
        }
    }

    @Override
    public void loop() {
        handleSlides();
        handleSlidePivots();
        handleArmServos();
        updateTelemetry();
    }

    private void handleSlides() {
        // PID-controlled slide extension
        if (gamepad2.dpad_up && !lastUp) {
            slideTarget = Math.min(slideTarget + TICK_INCREMENT, MAX_TICKS);
            lastUp = true;
        } else if (!gamepad2.dpad_up) {
            lastUp = false;
        }

        if (gamepad2.dpad_down && !lastDown) {
            slideTarget = Math.max(slideTarget - TICK_INCREMENT, 0);
            lastDown = true;
        } else if (!gamepad2.dpad_down) {
            lastDown = false;
        }

        // Calculate and apply PID power
        int currentPos = (slideLeft.getCurrentPosition() + slideRight.getCurrentPosition()) / 2;
        double power = slideController.calculate(currentPos, slideTarget) + f;
        slideLeft.setPower(power);
        slideRight.setPower(power);
    }

    private void handleSlidePivots() {
        // Manual pivot control using Y/A buttons
        controlMotorPower(gamepad2.y, slidePivotLeft, slidePivotRight, 0.6);
        controlMotorPower(gamepad2.a, slidePivotLeft, slidePivotRight, -0.6);
    }

    private void handleArmServos() {
        // Servo control logic
        updateServo(gamepad1.dpad_up, armclaw, 0.01);
        updateServo(gamepad1.dpad_down, armclaw, -0.01);
        updateServo(gamepad1.dpad_right, armpivot, 0.01);
        updateServo(gamepad1.dpad_left, armpivot, -0.01);
        updateServo(gamepad1.a, armpitch, -0.01);
        updateServo(gamepad1.y, armpitch, 0.01);
        updateServo(gamepad1.b, armwrist, 0.01);
        updateServo(gamepad1.x, armwrist, -0.01);
        updateServo(gamepad1.left_bumper, led, -0.01);
        updateServo(gamepad1.right_bumper, led, 0.01);
    }

    private void controlMotorPower(boolean buttonPressed, DcMotorEx motor1, DcMotorEx motor2, double power) {
        if (buttonPressed) {
            motor1.setPower(power);
            motor2.setPower(power);
        } else {
            motor1.setPower(0);
            motor2.setPower(0);
        }
    }

    private void updateServo(boolean condition, Servo servo, double delta) {
        if (condition) {
            double newPos = Math.min(Math.max(servo.getPosition() + delta, MIN_POSITION), MAX_POSITION);
            servo.setPosition(newPos);
            sleep(40);
        }
    }

    private void updateTelemetry() {
        telemetry.addData("Slide Target", slideTarget);
        telemetry.addData("Slide Position", (slideLeft.getCurrentPosition() + slideRight.getCurrentPosition()) / 2);
        telemetry.addData("Pivot Left Power", slidePivotLeft.getPower());
        telemetry.addData("Pivot Right Power", slidePivotRight.getPower());
        telemetry.addData("Pivot Position", (slidePivotLeft.getCurrentPosition() + slidePivotRight.getCurrentPosition()) / 2);

        telemetry.addData("armclaw", armclaw.getPosition());
        telemetry.addData("armpitch", armpitch.getPosition());
        telemetry.addData("armpivot", armpivot.getPosition());
        telemetry.addData("armwrist", armwrist.getPosition());
        telemetry.update();
    }

    private void sleep(int millis) {
        try {
            Thread.sleep(millis);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}

