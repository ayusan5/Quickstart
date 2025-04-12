//package org.firstinspires.ftc.teamcodeworlds.common.util.tests;
//
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//
//@TeleOp(name="Slide Test (Fixed)", group="Test")
//public class hang extends OpMode {
//    private DcMotorEx slidePivotLeft, slidePivotRight, slideLeft, slideRight;
//
//    // State machine variables
//    private enum ArmState { IDLE, MOVING_PIVOT, MOVING_SLIDES, COMPLEX_MOVE }
//    private ArmState currentState = ArmState.IDLE;
//    private boolean sequenceStarted = false;
//    private int targetPivotPos = 0;
//    private int targetSlidePos = 0;
//
//    @Override
//    public void init() {
//        // Motor initialization
//        slidePivotLeft = hardwareMap.get(DcMotorEx.class, "slidepivotLeft");
//        slidePivotRight = hardwareMap.get(DcMotorEx.class, "slidepivotRight");
//        slideLeft = hardwareMap.get(DcMotorEx.class, "slideLeft");
//        slideRight = hardwareMap.get(DcMotorEx.class, "slideRight");
//
//        // Motor configuration
//        slideRight.setDirection(DcMotor.Direction.REVERSE);
//        configureMotor(slidePivotLeft);
//        configureMotor(slidePivotRight);
//        configureMotor(slideLeft);
//        configureMotor(slideRight);
//    }
//
//    private void configureMotor(DcMotorEx motor) {
//        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        motor.setPower(0);
//    }
//
//    @Override
//    public void loop() {
//        handleGamepadInputs();
//        updateMovements();
//        updateTelemetry();
//    }
//
//    private void handleGamepadInputs() {
//        if (currentState != ArmState.IDLE) return;
//
//        if (gamepad2.a) {
//            startSimpleSequence(680, 1500);  // A button positions
//        }
//        else if (gamepad2.y) {
//            currentState = ArmState.COMPLEX_MOVE;
//            sequenceStarted = false;
//        }
//    }
//
//    private void startSimpleSequence(int pivotPos, int slidePos) {
//        targetPivotPos = pivotPos;
//        targetSlidePos = slidePos;
//        currentState = ArmState.MOVING_PIVOT;
//        sequenceStarted = false;
//    }
//
//    private void updateMovements() {
//        switch (currentState) {
//            case IDLE:
//                break;
//
//            case MOVING_PIVOT:
//                if (!sequenceStarted) {
//                    setPivotTarget(targetPivotPos, 1.0);
//                    sequenceStarted = true;
//                }
//                if (pivotsAtTarget()) {
//                    currentState = ArmState.MOVING_SLIDES;
//                }
//                break;
//
//            case MOVING_SLIDES:
//                setSlideTarget(targetSlidePos, 1.0);
//                currentState = ArmState.IDLE;
//                break;
//
//            case COMPLEX_MOVE:
//                handleComplexSequence();
//                break;
//        }
//    }
//
//    private void handleComplexSequence() {
//        if (!sequenceStarted) {
//            // Stage 1: Initial pivot movement
//            setPivotTarget(500, -1.0);
//            sequenceStarted = true;
//        }
//
//        if (pivotsAtTarget()) {
//            if (slidePivotLeft.getTargetPosition() == 500) {
//                // Stage 2: Slide retraction and pivot adjustment
//                setSlideTarget(270, -1.0);
//                setPivotTarget(400, -1.0);
//            }
//            else if (slidePivotLeft.getTargetPosition() == 400 & slideLeft.getTargetPosition() ==270 ) {
//                // Stage 3: Pivot elevation
//                setSlideTarget(270, -1.0);
//                setPivotTarget(1150, 1.0);
//            }
//            else if (slidePivotLeft.getTargetPosition() == 1150 && slidesAtTarget()) {
//                // Stage 4: Final slide adjustment
//                setSlideTarget(450, 1.0);
//                currentState = ArmState.IDLE;
//            }
//        }
//    }
//
//    private void setPivotTarget(int position, double power) {
//        slidePivotLeft.setTargetPosition(position);
//        slidePivotRight.setTargetPosition(position);
//        slidePivotLeft.setPower(Math.abs(power));
//        slidePivotRight.setPower(Math.abs(power));
//    }
//
//    private void setSlideTarget(int position, double power) {
//        slideLeft.setTargetPosition(position);
//        slideRight.setTargetPosition(position);
//        slideLeft.setPower(Math.abs(power));
//        slideRight.setPower(Math.abs(power));
//    }
//
//    private boolean pivotsAtTarget() {
//        return !slidePivotLeft.isBusy() && !slidePivotRight.isBusy();
//    }
//
//    private boolean slidesAtTarget() {
//        return !slideLeft.isBusy() && !slideRight.isBusy();
//    }
//
//    private void updateTelemetry() {
//        telemetry.addLine("SLIDE CONTROL SYSTEM");
//        telemetry.addData("Current State", currentState);
//        telemetry.addLine("\nMotor Positions:");
//        telemetry.addData("Left Pivot", slidePivotLeft.getCurrentPosition());
//        telemetry.addData("Right Pivot", slidePivotRight.getCurrentPosition());
//        telemetry.addData("Left Slide", slideLeft.getCurrentPosition());
//        telemetry.addData("Right Slide", slideRight.getCurrentPosition());
//        telemetry.addLine("\nTarget Positions:");
//        telemetry.addData("Pivot Target", slidePivotLeft.getTargetPosition());
//        telemetry.addData("Slide Target", slideLeft.getTargetPosition());
//        telemetry.update();
//    }
//}

package common.util.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name="Slide Test (Fixed)", group="Test")
public class hang extends OpMode {
    private DcMotorEx slidePivotLeft, slidePivotRight, slideLeft, slideRight;

    // Updated state machine variables
    private enum ArmState {IDLE, MOVING_PIVOT, MOVING_SLIDES, COMPLEX_MOVE, X_MOVE, B_MOVE}

    private ArmState currentState = ArmState.IDLE;
    private boolean sequenceStarted = false;
    private int targetPivotPos = 0;
    private int targetSlidePos = 0;

    @Override
    public void init() {
        // Motor initialization (unchanged)
        slidePivotLeft = hardwareMap.get(DcMotorEx.class, "slidepivotLeft");
        slidePivotRight = hardwareMap.get(DcMotorEx.class, "slidepivotRight");
        slideLeft = hardwareMap.get(DcMotorEx.class, "slideLeft");
        slideRight = hardwareMap.get(DcMotorEx.class, "slideRight");

        // Motor configuration (unchanged)
        slideRight.setDirection(DcMotor.Direction.REVERSE);
        configureMotor(slidePivotLeft);
        configureMotor(slidePivotRight);
        configureMotor(slideLeft);
        configureMotor(slideRight);
    }

    // configureMotor remains unchanged
    private void configureMotor(DcMotorEx motor) {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(0);
    }

    @Override
    public void loop() {
        handleGamepadInputs();
        updateMovements();
      updateTelemetry();
    }

    private void handleGamepadInputs() {
        if (currentState != ArmState.IDLE) return;

        if (gamepad2.a) {
            startSimpleSequence(680, 1500);
        } else if (gamepad2.y) {
            currentState = ArmState.COMPLEX_MOVE;
            sequenceStarted = false;
        } else if (gamepad2.x) {  // New X handler
            currentState = ArmState.X_MOVE;
            sequenceStarted = false;
        } else if (gamepad2.b) {  // New B handler
            currentState = ArmState.B_MOVE;
            sequenceStarted = false;
        }
    }

    // Added new movement handlers
    private void updateMovements() {
        switch (currentState) {
            case IDLE:
                break;

            case MOVING_PIVOT:
                handlePivotMovement();
                break;

            case MOVING_SLIDES:
                handleSlideMovement();
                break;

            case COMPLEX_MOVE:
                handleComplexSequence();
                break;

            case X_MOVE:  // New X case
                handleXSequence();
                break;

            case B_MOVE:  // New B case
                handleBSequence();
                break;
        }
    }

    // New X sequence handler
    private void handleXSequence() {
        if (!sequenceStarted) {
            // First stage: Extend slides
            setSlideTarget(2000, 1.0);
            sequenceStarted = true;
        } else if (slidesAtTarget()) {
            // Second stage: Move pivots
            setPivotTarget(975, 1.0);
            if (pivotsAtTarget()) {
                currentState = ArmState.IDLE;
                sequenceStarted = false;
            }
        }
    }

    // New B sequence handler
    private void handleBSequence() {
        if (!sequenceStarted) {
            // First stage: Retract pivots
            setPivotTarget(975, 1.0);
            sequenceStarted = true;
        } else if (pivotsAtTarget()) {
            // Second stage: Retract slides
            setSlideTarget(350, 1.0);
            if (slidesAtTarget()) {
                currentState = ArmState.IDLE;
                sequenceStarted = false;
            }
        }
    }

    // Existing complex sequence handler (modified for clarity)
    private void handleComplexSequence() {
        if (!sequenceStarted) {
            setPivotTarget(560, -1.0);
            sequenceStarted = true;
        }

        if (pivotsAtTarget()) {
            if (slidePivotLeft.getTargetPosition() == 560) {
                setSlideTarget(380, -1.0);
                setPivotTarget(500, -1.0);
            } else if (slidePivotLeft.getTargetPosition() == 500 && slideLeft.getTargetPosition() == 380) {
                setSlideTarget(215, -1.0);
                setPivotTarget(1300, 1.0);
            } else if (slidePivotLeft.getTargetPosition() == 1300 && slideLeft.getTargetPosition() == 215) {
                setSlideTarget(450, 1.0);
                setPivotTarget(1040, 1.0);
                currentState = ArmState.IDLE;
            }
        }
    }

    // Helper methods remain unchanged
    private void startSimpleSequence(int pivotPos, int slidePos) {
        targetPivotPos = pivotPos;
        targetSlidePos = slidePos;
        currentState = ArmState.MOVING_PIVOT;
        sequenceStarted = false;
    }

    private void handlePivotMovement() {
        if (!sequenceStarted) {
            setPivotTarget(targetPivotPos, 1.0);
            sequenceStarted = true;
        }
        if (pivotsAtTarget()) {
            currentState = ArmState.MOVING_SLIDES;
        }
    }

    private void handleSlideMovement() {
        setSlideTarget(targetSlidePos, 1.0);
        currentState = ArmState.IDLE;
    }

    private void setPivotTarget(int position, double power) {
        slidePivotLeft.setTargetPosition(position);
        slidePivotRight.setTargetPosition(position);
        slidePivotLeft.setPower(Math.abs(power));
        slidePivotRight.setPower(Math.abs(power));
    }

    private void setSlideTarget(int position, double power) {
        slideLeft.setTargetPosition(position);
        slideRight.setTargetPosition(position);
        slideLeft.setPower(Math.abs(power));
        slideRight.setPower(Math.abs(power));
    }

    private boolean pivotsAtTarget() {
        return !slidePivotLeft.isBusy() && !slidePivotRight.isBusy();
    }

    private boolean slidesAtTarget() {
        return !slideLeft.isBusy() && !slideRight.isBusy();
    }



    private void updateTelemetry() {
        telemetry.addLine("SLIDE CONTROL SYSTEM");
        telemetry.addData("Current State", currentState);
        telemetry.addLine("\nMotor Positions:");
        telemetry.addData("Left Pivot", slidePivotLeft.getCurrentPosition());
        telemetry.addData("Right Pivot", slidePivotRight.getCurrentPosition());
        telemetry.addData("Left Slide", slideLeft.getCurrentPosition());
        telemetry.addData("Right Slide", slideRight.getCurrentPosition());
        telemetry.addLine("\nTarget Positions:");
        telemetry.addData("Pivot Target", slidePivotLeft.getTargetPosition());
        telemetry.addData("Slide Target", slideLeft.getTargetPosition());
        telemetry.update();
    }
}

// Telemetry method remains unchanged