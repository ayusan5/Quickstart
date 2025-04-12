package opmode.blue;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import common.IntoTheDeepConstants;
import common.commands.arm.armClawCommands.armClawGrabPos;
import common.commands.arm.armClawCommands.armClawReleasePos;
import common.commands.arm.armPitchCommands.armPitchBarrierPos;
import common.commands.arm.armPitchCommands.armPitchCenterPos;
import common.commands.arm.armPitchCommands.armPitchSampleDepositPos;
import common.commands.arm.armPitchCommands.armPitchSampleGrabPos;
import common.commands.arm.armPitchCommands.armPitchSpecimenDepositPos;
import common.commands.arm.armPitchCommands.armPitchSpecimenGrabPos;
import common.commands.arm.armPivotCommands.armPivotBarrierPos;
import common.commands.arm.armPivotCommands.armPivotInitPos;
import common.commands.arm.armPivotCommands.armPivotSampleGrabPos;
import common.commands.arm.armPivotCommands.armPivotSpecimenGrabPos;
import common.commands.arm.armWristCommands.armWristCenterPos;
import common.commands.arm.armWristCommands.armWristScan;
import common.commands.arm.armWristCommands.armWristScanStatePos;
import common.commands.arm.armWristCommands.armWristSpecimenDepositPos;
import common.commands.slide.setSlideTargetPosHor;
import common.commands.slide.setSlideTargetPosVer;
import common.commands.slidePivot.slidePivotSetPos;
import common.robot.robotConstants;
import common.robot.robotHardware;

import java.util.function.BooleanSupplier;

@TeleOp(name="Tele Blue Sample", group="TeleOp")
public class tele extends CommandOpMode {

    private final robotHardware robot = robotHardware.getInstance();
    private GamepadEx gamepadDrivetrain;
    private GamepadEx gamepadMechanism;
    private BooleanSupplier slideSampleDeposit = () -> robot.slideMotor.getPosition() > 2000;
    private BooleanSupplier slideSampleAfterDeposit = () -> robot.slideMotor.getPosition() < 10;
    private BooleanSupplier slidePivotSpecimenToDeposit = () -> robot.slidePivotMotor.getPosition() < 410;
    private BooleanSupplier slidePivotSpecimenToGrab = () -> robot.slidePivotMotor.getPosition() > 1000;

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

        if (gamepadMechanism.getGamepadButton(GamepadKeys.Button.A).get()){
            CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new setSlideTargetPosHor(robot.slideSub.slideTargetPos-40),
                        new WaitCommand(125)
                )
                );
        }
        if (gamepadMechanism.getGamepadButton(GamepadKeys.Button.Y).get()){
            CommandScheduler.getInstance().schedule(
            new SequentialCommandGroup(
                    new setSlideTargetPosHor(robot.slideSub.slideTargetPos+40),
                    new WaitCommand(125)
            )
            );
        }

        if (gamepadMechanism.getGamepadButton(GamepadKeys.Button.B).get()){
            CommandScheduler.getInstance().schedule(
                    new SequentialCommandGroup(
                            new setSlideTargetPosHor(0),
                            new WaitUntilCommand(slideSampleAfterDeposit),
                            new slidePivotSetPos(1040)
                            )
            );
        }
        if (gamepadMechanism.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON).get()) {
            if (robot.slideMotor.getPosition() > 700) {
                CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new armPivotBarrierPos(),
                        new armPitchBarrierPos(),
                        new armWristCenterPos()
//                        new armClawReleasePos()
                )
                );
            }
        }

        if (gamepadMechanism.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).get()){
            if (robot.armSub.armWristServoPos>=0) {
                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                new armWristScanStatePos(),
                                new armWristScan(-robotConstants.armWristMovementSpeed),
                                new WaitCommand(75)
                        )
                );
            } else if (robot.armWristServo.getPosition()<=0) {
                robot.armSub.armWristServoPos=0;
            }
        }

        if (gamepadMechanism.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).get()){
            if (robot.armSub.armWristServoPos<=1) {
                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                new armWristScanStatePos(),
                                new armWristScan(robotConstants.armWristMovementSpeed),
                                new WaitCommand(75)
                        )
                );
            } else if (robot.armWristServo.getPosition()>=1) {
                robot.armSub.armWristServoPos=0;
            }
        }

        if (gamepadDrivetrain.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).get()){
            CommandScheduler.getInstance().schedule(new slidePivotSetPos(0));
        }

        if (gamepadDrivetrain.getGamepadButton(GamepadKeys.Button.DPAD_UP).get()){
            CommandScheduler.getInstance().schedule(new slidePivotSetPos(1040));
        }

        if (gamepadDrivetrain.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).get()){
            CommandScheduler.getInstance().schedule(new slidePivotSetPos(380));
        }

        if (gamepadMechanism.getGamepadButton(GamepadKeys.Button.DPAD_UP).get()){
            CommandScheduler.getInstance().schedule(
                    new SequentialCommandGroup(
                            new setSlideTargetPosVer(2050),
                            new WaitUntilCommand(slideSampleDeposit),
                            new armPitchCenterPos()
                    )
            );
        }

        if (gamepadMechanism.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).get()){
            CommandScheduler.getInstance().schedule(
                    new SequentialCommandGroup(
                            new armPitchSampleDepositPos(),
                            new WaitCommand(85),
                            new armClawReleasePos(),
                            new WaitCommand(50),
                            new armPitchCenterPos(),
                            new setSlideTargetPosVer(0),
                            new WaitUntilCommand(slideSampleAfterDeposit),
                            new slidePivotSetPos(0)
                    )
            );
        }

        if (gamepadDrivetrain.getGamepadButton(GamepadKeys.Button.A).get()){
            CommandScheduler.getInstance().schedule(
                    new SequentialCommandGroup(
                            new armClawReleasePos(),
                            new armPivotInitPos(),
                            new armPitchCenterPos(),
                            new armWristCenterPos(),
                            new WaitCommand(100),
                            new setSlideTargetPosHor(0),
                            new WaitUntilCommand(slideSampleAfterDeposit),
                            new slidePivotSetPos(1040),
                            new WaitUntilCommand(slidePivotSpecimenToGrab),
                            new WaitCommand(100),
                            new armPivotSpecimenGrabPos(),
                            new armPitchSpecimenGrabPos()
                    )
            );
        }

        if (gamepadDrivetrain.getGamepadButton(GamepadKeys.Button.Y).get()){
            CommandScheduler.getInstance().schedule(
                    new SequentialCommandGroup(
                            new armClawGrabPos(),
                            new WaitCommand(100),
                            new armPivotInitPos(),
                            new armPitchSpecimenDepositPos(),
                            new armWristSpecimenDepositPos(),
                            new slidePivotSetPos(380),
                            new WaitUntilCommand(slidePivotSpecimenToDeposit),
                            new setSlideTargetPosHor(1200)
                    )
            );
        }
        if (gamepadMechanism.getGamepadButton(GamepadKeys.Button.X).get()){
            CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new setSlideTargetPosHor(robot.slideSub.slideTargetPos+25),
                        new WaitCommand(100),
                        new armPivotSampleGrabPos(),
                        new armPitchSampleGrabPos(),
                        new WaitCommand(175),
                        new armClawGrabPos(),
                        new WaitCommand(175),
                        new armPivotBarrierPos(),
//                        new armWristCenterPos(),
                        new armPitchBarrierPos()
                )
        );
        }


//        if (gamepadMechanism.getGamepadButton(GamepadKeys.Button.X).get()){
//            new armClawGrabPos();
//            new WaitCommand(75);
//            if (robot.colorsensor.blue()>robot.colorsensor.red()){
//                new SequentialCommandGroup(
//                        new setSlideTargetPos(0),
//                        new armPivotInitPos(),
//                        new armWristCenterPos(),
//                        new armPitchCenterPos()
//                );
//            }
//            else{
//                new armClawReleasePos();
//            }
//        }

        telemetry.addData("Left Front Power", leftFrontPower);
        telemetry.addData("Left Rear Power", leftRearPower);
        telemetry.addData("Right Front Power", rightFrontPower);
        telemetry.addData("Right Rear Power", rightRearPower);
        telemetry.addData("slidepos", robot.slideMotor.getPosition());
        telemetry.addData("slidepos target", robot.slideSub.slideTargetPos);

        telemetry.update();
    }
}
