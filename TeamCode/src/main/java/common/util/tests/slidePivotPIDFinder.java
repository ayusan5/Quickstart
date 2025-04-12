package common.util.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Config
@Disabled
public class slidePivotPIDFinder extends OpMode {

    private PIDController controller;

    public static double p=0.015 , i=0, d=0.0005;
    public static double f=0;

    public static double target = 0;

    private DcMotorEx slidePivotLeft;
    private DcMotorEx slidePivotRight;
    private Servo armclaw, armwrist, armpitch, armpivot, led;

    @Override
    public void init() {
        controller = new PIDController(p,i,d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        slidePivotLeft = hardwareMap.get(DcMotorEx.class, "slidepivotLeft");
        slidePivotRight = hardwareMap.get(DcMotorEx.class, "slidepivotRight");

        slidePivotRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidePivotRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slidePivotLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidePivotLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armclaw = hardwareMap.get(Servo.class, "armclawServo");
        armwrist = hardwareMap.get(Servo.class, "armwristServo");
        armpitch = hardwareMap.get(Servo.class, "armpitchServo");
        armpivot = hardwareMap.get(Servo.class, "armpivotServo");
        led = hardwareMap.get(Servo.class, "ledflashServo");
        armclaw.setPosition(0);
        armpivot.setPosition(0.34);
        armpitch.setPosition(0.45);
        armwrist.setPosition(0.42);
        led.setPosition(0);

//        slideRight.setDirection(DcMotorEx.Direction.REVERSE);
//        slideLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        controller.setPID(p,i,d);
        int armPos1 = slidePivotLeft.getCurrentPosition();
        int armPos2 = slidePivotRight.getCurrentPosition();
        double pid = controller.calculate((armPos1+armPos2)/2, target);
        double power = pid + f;

        slidePivotLeft.setPower(power);
        slidePivotRight.setPower(power);

        telemetry.addData("pos1 ", armPos1);
        telemetry.addData("pos2 ", armPos2);
        telemetry.addData("target", target);
        telemetry.update();
    }
}
