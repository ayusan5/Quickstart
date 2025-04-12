package common.subsystems;

//import common.robot.robotConstants;
import common.util.wrappers.MESubsystem;
//import common.robot.robotHardware;

import org.jetbrains.annotations.NotNull;

import common.robot.robotConstants;
import common.robot.robotHardware;

public class armSubsystem extends MESubsystem {
    robotHardware robot = robotHardware.getInstance();
    public double armClawServoPos;
    public double armWristServoPos;
    public double armPitchServoPos;
    public double armPivotServoPos;

    public enum armClawState{
        armClawGrabPos,
        armClawReleasePos,
    }
    public enum armWristState{
        armWristCenterPos,
        armWristScanState,
        armWristSpecimenDepositPos,
        armWristNoMove
    }
    public enum armPitchState{
        armPitchCenterPos,
        armPitchSampleGrabPos,
        armPitchSampleDepositPos,
        armPitchSpecimenGrabPos,
        armPitchSpecimenDepositPos,
        armPitchBarrierPos,
        armPitchNoMove
    }
    public enum armPivotState{
        armPivotInitPos,
        armPivotSampleGrabPos,
        armPivotBarrierPos,
        armPivotSpecimenGrabPos,
        armPivotSpecimenDepositPos,
        armPivotNoMove
    }

    armClawState clawState = armClawState.armClawReleasePos;
    armWristState wristState = armWristState.armWristCenterPos;
    armPitchState pitchState = armPitchState.armPitchNoMove;
    armPivotState pivotState = armPivotState.armPivotNoMove;

    public armSubsystem(){}
    public void read(){}

    public void periodic(){
        switch (clawState){
            case armClawReleasePos:
                armClawServoPos = robotConstants.armClawReleasePos;
                break;
            case armClawGrabPos:
                armClawServoPos = robotConstants.armClawGrabPos;
                break;
        }
        switch (wristState){
            case armWristCenterPos:
                armWristServoPos = robotConstants.armWristCenterPos;
                break;
            case armWristScanState:
                break;
            case armWristSpecimenDepositPos:
                armWristServoPos = robotConstants.armWristSpecimenDepositPos;
                break;
            case armWristNoMove:
                break;
        }
        switch (pitchState){
            case armPitchCenterPos:
                armPitchServoPos = robotConstants.armPitchCenterPos;
                break;
            case armPitchSampleGrabPos:
                armPitchServoPos = robotConstants.armPitchSampleGrabPos;
                break;
            case armPitchSampleDepositPos:
                armPitchServoPos = robotConstants.armPitchSampleDepositPos;
                break;
            case armPitchSpecimenGrabPos:
                armPitchServoPos = robotConstants.armPitchSpecimenGrabPos;
                break;
            case armPitchSpecimenDepositPos:
                armPitchServoPos = robotConstants.armPitchSpecimenDepositPos;
                break;
            case armPitchBarrierPos:
                armPitchServoPos = robotConstants.armPitchBarrierPos;
                break;
            case armPitchNoMove:
                break;
        }
        switch (pivotState) {
            case armPivotInitPos:
                armPivotServoPos = robotConstants.armPivotInitPos;
                break;
            case armPivotSampleGrabPos:
                armPivotServoPos = robotConstants.armPivotSampleGrabPos;
                break;
            case armPivotBarrierPos:
                armPivotServoPos = robotConstants.armPivotBarrierPos;
                break;
            case armPivotSpecimenGrabPos:
                armPivotServoPos = robotConstants.armPivotSpecimenGrabPos;
                break;
            case armPivotSpecimenDepositPos:
                armPivotServoPos = robotConstants.armPivotSpecimenDepositPos;
                break;
            case armPivotNoMove:
                break;
        }
    }

    public void changearmWristServoPos(double value){
        if (armWristServoPos>0.05 && armWristServoPos<0.95) {
            armWristServoPos += value;
        }
    }
    public void write(){
        robot.armClawServo.setPosition(armClawServoPos);
        robot.armPivotServo.setPosition(armPivotServoPos);
        robot.armPitchServo.setPosition(armPitchServoPos);
        robot.armWristServo.setPosition(armWristServoPos);
    }
    public void reset(){
        clawState = armClawState.armClawReleasePos;
        wristState = armWristState.armWristCenterPos;
        pitchState = armPitchState.armPitchCenterPos;
        pivotState = armPivotState.armPivotInitPos;
    }
    public void updateClawState(@NotNull armSubsystem.armClawState state){
        this.clawState=state;
    }
    public void updateWristState(@NotNull armSubsystem.armWristState state){
        this.wristState=state;
    }
    public void updatePivotState(@NotNull armSubsystem.armPivotState state){
        this.pivotState=state;
    }
    public void updatePitchState(@NotNull armSubsystem.armPitchState state){
        this.pitchState=state;
    }

    public armSubsystem.armClawState getClawState(){
        return clawState;
    }
    public armSubsystem.armWristState getWristState(){
        return wristState;
    }
    public armSubsystem.armPivotState getPivotState(){
        return pivotState;
    }
    public armSubsystem.armPitchState getPitchState(){
        return pitchState;
    }
}
