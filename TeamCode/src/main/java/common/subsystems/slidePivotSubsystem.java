package common.subsystems;

import com.arcrobotics.ftclib.controller.PIDController;

import common.robot.robotConstants;
import common.robot.robotHardware;
import common.util.MathUtils;
import common.util.wrappers.MEActuator;
import common.util.wrappers.MESubsystem;

public class slidePivotSubsystem extends MESubsystem {
    robotHardware robot = robotHardware.getInstance();
    int slidePivotTargetPos = 0;
    PIDController slidePivotPIDController = robotConstants.slidePivotPIDController;
    double slidePivotFeedForward = robotConstants.slidePivotFF;

    public enum slidePivotState {
        slidePivotInitPos,
        slidePivotSampleDepositPos,
        slidePivotSampleGrabPos,
        slidePivotSpecimenDepositPos,
        slidePivotSpecimenGrabPos
    }

    slidePivotState slidepivotstate = slidePivotState.slidePivotInitPos;

    public slidePivotSubsystem() {
    }


    @Override
    public void read() {
        robot.slidePivotMotor.read();
    }

    @Override
    public void periodic() {
        slidePivotTargetPos = (int) MathUtils.clamp(slidePivotTargetPos, 0, robotConstants.slidePivotMaxLimit);
        robot.slidePivotMotor.setPIDController(slidePivotPIDController);
        robot.slidePivotMotor.setTargetPosition(slidePivotTargetPos);
        robot.slidePivotMotor.setFeedforward(MEActuator.FeedforwardMode.CONSTANT, slidePivotFeedForward);
        robot.slidePivotMotor.periodic();
    }

    @Override
    public void write() {
        robot.slidePivotMotor.write();
    }

    @Override
    public void reset() {
        slidePivotTargetPos = 0;
    }


    public void setSlidePivotTargetPos(int value){
        if (value >= 0 && value <= robotConstants.slidePivotMaxLimit) {
            slidePivotTargetPos = value;
        } else if (slidePivotTargetPos>=robotConstants.slidePivotMaxLimit) {
            slidePivotTargetPos = robotConstants.slidePivotMaxLimit;
        }
    }

    public void setSlidePivotPIDController (PIDController controller){
        slidePivotPIDController = controller;
    }

    public boolean isPivotDown(){
        return robot.slidePivotMotor.getPosition()<50;
    }

    public void setSlidePivotFeedForward (double value){
        slidePivotFeedForward = value;
    }
}