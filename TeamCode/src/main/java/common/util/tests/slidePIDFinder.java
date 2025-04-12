package common.util.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
@Config
//@Disabled
public class slidePIDFinder extends OpMode {

    private PIDController controller;

    public static double p=0.03 , i=0, d=0.00058;
    public static double f=0.01;

    public static double target = 0;

    private DcMotorEx slideLeft;
    private DcMotorEx slideRight;

    @Override
    public void init() {
        controller = new PIDController(p,i,d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        slideLeft = hardwareMap.get(DcMotorEx.class, "slideLeft");
        slideRight = hardwareMap.get(DcMotorEx.class, "slideRight");

        slideRight.setDirection(DcMotorSimple.Direction.REVERSE);

//        slideRight.setDirection(DcMotorEx.Direction.REVERSE);
//        slideLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        controller.setPID(p,i,d);
        int armPos1 = slideLeft.getCurrentPosition();
        int armPos2 = slideRight.getCurrentPosition();
        double pid = controller.calculate(armPos1, target);
        double power = pid + f;

        slideLeft.setPower(power);
        slideRight.setPower(power);



        telemetry.addData("pos1 ", armPos1);
        telemetry.addData("target", target);
        telemetry.update();
    }
}
