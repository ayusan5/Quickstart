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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Config
//@Disabled
public class slideBothPIDFinder extends OpMode {

    private PIDController cPivot;
    private PIDController cSlide;

    public static double p=0.01 , i=0.000001, d=0.0005;
    public static double f=0;

    public static double sp=0.02 ,si=0, sd=0.0005;
    public static double sf=0.07;

    public static double targetSlide = 0;

    private DcMotorEx slideLeft;
    private DcMotorEx slideRight;

    public static double targetPivot = 0;

    private DcMotorEx slidePivotLeft;
    private DcMotorEx slidePivotRight;
    private Servo armclaw, armwrist, armpitch, armpivot, led;

    @Override
    public void init() {
        cPivot = new PIDController(p,i,d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        cSlide = new PIDController(sp,si,sd);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        slideLeft = hardwareMap.get(DcMotorEx.class, "slideLeft");
        slideRight = hardwareMap.get(DcMotorEx.class, "slideRight");

        slideRight.setDirection(DcMotorSimple.Direction.REVERSE);

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
        cPivot.setPID(p,i,d);
        cSlide.setPID(sp,si,sd);
        int armPos1 = slidePivotLeft.getCurrentPosition();
        int armPos2 = slidePivotRight.getCurrentPosition();
        int slidepos = slideLeft.getCurrentPosition();
        double pid = cPivot.calculate((armPos1+armPos2)/2, targetPivot);
        double power = pid + f;
        double pidslide = cSlide.calculate(slidepos, targetSlide);
        double powerslide = pidslide + sf;

        slidePivotLeft.setPower(power);
        slidePivotRight.setPower(power);
        slideLeft.setPower(powerslide);
        slideRight.setPower(powerslide);

        telemetry.addData("pos1PIVOT ", armPos1);
        telemetry.addData("pos2 PIVOT", armPos2);
        telemetry.addData("targetPIVOT", targetPivot);

        telemetry.addData("pos ", slidepos);
        telemetry.addData("tarsldie", targetSlide);
        telemetry.update();
    }
}
