package common.util.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import common.robot.robotConstants;

@TeleOp
//@Disabled
public class finalBossTesting extends OpMode {

    private Servo armclaw;
    private Servo armwrist;
    private Servo armpitch;
    private Servo armpivot;
    private Servo led;
    public float looptime, previousTime, currentTime;

    public double sp=0.01 , si=0, sd=0.0001;
    public double sf=0.003;
    public double slidetarget = 0;

    private DcMotorEx slideLeft;
    private DcMotorEx slideRight;

    public static double spp=0.013 , spi=0, spd=0.000038;
    public static double spf=0.0026;

    public static double sptarget = 0;

    private PIDController scontroller;
    private PIDController spcontroller;


    private DcMotorEx slidePivotLeft;
    private DcMotorEx slidePivotRight;

    @Override
    public void init() {
        // Initialize the servo motor
        armclaw = hardwareMap.get(Servo.class, "armclawServo");
        armwrist = hardwareMap.get(Servo.class, "armwristServo");
        armpitch = hardwareMap.get(Servo.class, "armpitchServo");
        armpivot = hardwareMap.get(Servo.class, "armpivotServo");
        led = hardwareMap.get(Servo.class, "ledflashServo");

        // Set the initial position
        armpitch.setPosition(robotConstants.armPitchCenterPos);
        armclaw.setPosition(robotConstants.armClawReleasePos);
        armpivot.setPosition(robotConstants.armPivotInitPos);
        armwrist.setPosition(robotConstants.armWristCenterPos);
        led.setPosition(0);

        scontroller = new PIDController(sp,si,sd);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        slideLeft = hardwareMap.get(DcMotorEx.class, "slideLeft");
        slideRight = hardwareMap.get(DcMotorEx.class, "slideRight");

        slideRight.setDirection(DcMotorSimple.Direction.REVERSE);

        spcontroller = new PIDController(spp,spi,spd);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        slidePivotLeft = hardwareMap.get(DcMotorEx.class, "slidepivotLeft");
        slidePivotRight = hardwareMap.get(DcMotorEx.class, "slidepivotRight");

        slidePivotRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidePivotRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slidePivotLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidePivotLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        slideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
    @Override
    public void loop() {

        scontroller.setPID(sp,si,sd);
        int sarmPos1 = slideLeft.getCurrentPosition();
        int sarmPos2 = slideRight.getCurrentPosition();
        double spid = scontroller.calculate((sarmPos2+sarmPos1)/2, slidetarget);
        double spower = spid + sf;

        spcontroller.setPID(spp,spi,spd);
        int sparmPos1 = slidePivotLeft.getCurrentPosition();
        int sparmPos2 = slidePivotRight.getCurrentPosition();
        double sppid = spcontroller.calculate((sparmPos1+sparmPos2)/2, sptarget);
        double sppower = sppid + spf;

        slideLeft.setPower(spower);
        slideRight.setPower(spower);

        slidePivotLeft.setPower(sppower);
        slidePivotRight.setPower(sppower);

        // Increase the servo position when D-Pad up is pressed with debounce
        if (gamepad1.dpad_up){
            slidetarget += 20;
            sleep(40);
        }
        if (gamepad1.dpad_down){
            slidetarget -= 20;
            sleep(40);
        }
        if (gamepad2.dpad_right){
            sptarget += 20;
            sleep(100);
        }
        if (gamepad2.dpad_left){
            sptarget -= 20;
            sleep(100);
        }
        if (gamepad1.left_stick_button){
            armclaw.setPosition(armclaw.getPosition()-0.01);
            sleep(40);
        }
        if (gamepad1.right_stick_button){
            armclaw.setPosition(armclaw.getPosition()+0.01);
            sleep(40);
        }
        if (gamepad1.dpad_right){
            armpivot.setPosition(armpivot.getPosition()+0.01);
            sleep(40);
        }
        if (gamepad1.dpad_left){
            armpivot.setPosition(armpivot.getPosition()-0.01);
            sleep(40);
        }
        if (gamepad1.a){
            armpitch.setPosition(armpitch.getPosition()-0.01);
            sleep(40);
        }
        if (gamepad1.y){
            armpitch.setPosition(armpitch.getPosition()+0.01);
            sleep(40);
        }
        if (gamepad1.b){
            armwrist.setPosition(armwrist.getPosition()+0.01);
            sleep(40);
        }
        if (gamepad1.x){
            armwrist.setPosition(armwrist.getPosition()-0.01);
            sleep(40);
        }
        if (gamepad1.left_bumper){
            led.setPosition(led.getPosition()-0.01);
            sleep(40);
        }
        if (gamepad1.right_bumper) {
            led.setPosition(led.getPosition() + 0.01);
            sleep(40);
        }
        // Set the servo to the updated position
        currentTime=System.nanoTime();
        looptime = currentTime-previousTime;
        previousTime= currentTime;
        telemetry.addData("looptime",looptime);
        // Telemetry data for debugging
        telemetry.addData("armclaw", armclaw.getPosition());
        telemetry.addData("armpitch", armpitch.getPosition());
        telemetry.addData("armpivot", armpivot.getPosition());
        telemetry.addData("armwrist", armwrist.getPosition());
        telemetry.addData("led", led.getPosition());
        telemetry.addData("slidepivot", sptarget);
        telemetry.addData("slide", slidetarget);
        telemetry.addData("spl enc", slidePivotLeft.getCurrentPosition());
        telemetry.addData("spr enc", slidePivotRight.getCurrentPosition());
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
