package common.util.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="Chassis Control", group = "TeleOp")
@Disabled
public class chassisControl extends OpMode {

    private DcMotorEx leftFront;
    private DcMotorEx leftRear;
    private DcMotorEx rightFront;
    private DcMotorEx rightRear;

    @Override
    public void init() {
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");

        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        double drive = gamepad1.left_stick_y; // Forward and backward
        double strafe = -gamepad1.left_stick_x; // Strafing
        double turn = (gamepad1.left_trigger - gamepad1.right_trigger) * 0.8; // Turning

        double leftFrontPower = (drive + strafe + turn) * 1;
        double leftRearPower = (drive - strafe + turn) * 1;
        double rightFrontPower = (drive - strafe - turn) * 1;
        double rightRearPower = (drive + strafe - turn) * 1;

        leftFront.setPower(leftFrontPower);
        leftRear.setPower(leftRearPower);
        rightFront.setPower(rightFrontPower);
        rightRear.setPower(rightRearPower);

        telemetry.addData("Left Front Power", leftFrontPower);
        telemetry.addData("Left Rear Power", leftRearPower);
        telemetry.addData("Right Front Power", rightFrontPower);
        telemetry.addData("Right Rear Power", rightRearPower);

        telemetry.update();
    }
}
