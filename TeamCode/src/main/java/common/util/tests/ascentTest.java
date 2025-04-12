package common.util.tests;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.robocol.Command;

import common.IntoTheDeepConstants;
import common.commands.arm.armClawCommands.armClawGrabPos;
import common.commands.arm.armClawCommands.armClawReleasePos;
import common.commands.arm.armPitchCommands.armPitchBarrierPos;
import common.commands.arm.armPitchCommands.armPitchCenterPos;
import common.commands.arm.armPitchCommands.armPitchSampleGrabPos;
import common.commands.arm.armPivotCommands.armPivotBarrierPos;
import common.commands.arm.armPivotCommands.armPivotInitPos;
import common.commands.arm.armPivotCommands.armPivotSampleGrabPos;
import common.commands.arm.armWristCommands.armWristCenterPos;
import common.commands.slide.setHang;
import common.commands.slide.setSlideTargetPosVer;
import common.commands.slidePivot.slidePivotSetPos;
import common.robot.robotHardware;

import java.util.function.BooleanSupplier;

@TeleOp(name = "ascentTest", group = "TeleOp")
//@Disabled
public class ascentTest extends CommandOpMode {
    private final robotHardware robot = robotHardware.getInstance();
    private GamepadEx gamepadDrivetrain;
    private GamepadEx gamepadMechanism;
    private BooleanSupplier slideSampleDeposit = () -> robot.slideMotor.getPosition() > 2000;
    private BooleanSupplier slidePivotAscent = () -> robot.slideMotor.getPosition() > 1500;
    private BooleanSupplier slidePivotAscentLevelTwo = () -> robot.slideMotor.getPosition() > 1500;
    private BooleanSupplier slideDown = () -> robot.slideMotor.getPosition() < 285;

    private BooleanSupplier slideSampleAfterDeposit = () -> robot.slideMotor.getPosition() < 10;
    private BooleanSupplier slidePivotSpecimenToDeposit = () -> robot.slidePivotMotor.getPosition() < 390;
    private BooleanSupplier slidePivotSpecimenToGrab = () -> robot.slidePivotMotor.getPosition() > 1080;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        IntoTheDeepConstants.IS_AUTO = false;
//        robot.follower.setAuto(IntoTheDeepConstants.IS_AUTO);
        gamepadDrivetrain = new GamepadEx(gamepad1);
        gamepadMechanism = new GamepadEx(gamepad2);
        robot.init(hardwareMap);

//        gamepadMechanism.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
//                new SequentialCommandGroup(
//                        // TODO: Slide Extend
//                )
//        );
//        gamepadMechanism.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
//                new SequentialCommandGroup(
//                        // TODO: Slide Retract
//                )
//        );
//
        gamepadMechanism.getGamepadButton(GamepadKeys.Button.X).whenPressed(
                new SequentialCommandGroup(
                        new armPivotSampleGrabPos(),
                        new armPitchSampleGrabPos(),
                        new WaitCommand(175),
                        new armClawGrabPos(),
                        new WaitCommand(175),
                        new armPivotBarrierPos(),
                        new armWristCenterPos(),
                        new armPitchBarrierPos()
                )
        );
//
//        gamepadMechanism.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(
//                new SequentialCommandGroup(
//                        // TODO: Slide Full Extend till deposit basket pos commands
//                )
//        );
//
//        gamepadMechanism.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(
//                new SequentialCommandGroup(
//                        // TODO: Slide Full Retract after depositing sample commands
//                )
//        );

        gamepadMechanism.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON).whenPressed(
                new armClawReleasePos()
        );



        gamepadMechanism.getGamepadButton(GamepadKeys.Button.B).whenPressed(
                new SequentialCommandGroup(
                        new armPivotInitPos(),
                        new armPitchCenterPos(),
                        new armWristCenterPos()
//                        new armClawReleasePos()
                )
        );
        robot.ledFlashServo.setPosition(0);
        robot.read();
        robot.periodic();
        robot.write();
        while (opModeInInit()) {
            robot.ledFlashServo.setPosition(0);
            telemetry.addLine("Robot Initialized.");
            telemetry.update();
        }
    }
    @Override
    public void run(){
        CommandScheduler.getInstance().run();
        robot.read();
        robot.periodic();
        robot.write();

        double drive = -gamepadDrivetrain.getLeftY(); // Forward and backward
        double strafe = -gamepadDrivetrain.getLeftX(); // Strafing
        double turn = (gamepadDrivetrain.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) - gamepadDrivetrain.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) * 0.8); // Turning

        double leftFrontPower = (drive + strafe + turn) * 1;
        double leftRearPower = (drive - strafe + turn) * 1;
        double rightFrontPower = (drive - strafe - turn) * 1;
        double rightRearPower = (drive + strafe - turn) * 1;

        robot.leftFront.setPower(leftFrontPower);
        robot.leftRear.setPower(leftRearPower);
        robot.rightFront.setPower(rightFrontPower);
        robot.rightRear.setPower(rightRearPower);

        if (gamepadDrivetrain.getButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)){
            CommandScheduler.getInstance().schedule(
                    new SequentialCommandGroup(
                            new armPitchCenterPos(),
                            new armPivotInitPos(),
                            new slidePivotSetPos(680),
//                            new WaitUntilCommand(slidePivotAscent),
                            new WaitCommand(1000),
                            new setSlideTargetPosVer(1520)
                    )
            );
        };
        if (gamepadDrivetrain.getButton(GamepadKeys.Button.DPAD_DOWN)){
            CommandScheduler.getInstance().schedule(
                    new SequentialCommandGroup(
                            new slidePivotSetPos(560),
                            new WaitUntilCommand(slidePivotAscentLevelTwo),
                            new setSlideTargetPosVer(1520),

                            new WaitCommand(1000),

                            new setHang(true),
                            new setSlideTargetPosVer(280),
                            new WaitCommand(1500),
                            new slidePivotSetPos(1300),
                            new WaitUntilCommand(slideDown),
                            new setHang(false),
                            new slidePivotSetPos(1040),
                            new WaitCommand(2000),
                            new setSlideTargetPosVer(1990)



                    )
            );
        }
        }

    }



