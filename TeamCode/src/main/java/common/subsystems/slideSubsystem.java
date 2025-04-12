package common.subsystems;

import com.arcrobotics.ftclib.controller.PIDController;

import common.robot.robotConstants;
import common.robot.robotHardware;
import common.util.MathUtils;
import common.util.wrappers.MEActuator;
import common.util.wrappers.MESubsystem;

public class slideSubsystem extends MESubsystem {
    robotHardware robot = robotHardware.getInstance();
    public int slideTargetPos;
    PIDController slidePIDController = robotConstants.slidePIDController;
    boolean hangOn;

    public slideSubsystem(){
        hangOn=false;
    }

    @Override
    public void read(){
        robot.slideMotor.read();
    }

    @Override
    public void periodic(){
        slideTargetPos = (int)MathUtils.clamp(slideTargetPos, 0, robotConstants.slideMaxVerLimit);
        robot.slideMotor.setPIDController(slidePIDController);
        robot.slideMotor.setTargetPosition(slideTargetPos);
        if (!robot.slidePivotSub.isPivotDown()) robot.slideMotor.setFeedforward(MEActuator.FeedforwardMode.CONSTANT, robotConstants.slideFF);
        if (hangOn) robot.slideMotor.setFeedforward(MEActuator.FeedforwardMode.CONSTANT, robotConstants.slideHangFF);
        robot.slideMotor.periodic();
    }

    @Override
    public void write(){
        robot.slideMotor.write();
    }

    @Override
    public void reset(){
        slideTargetPos = 0;
    }

    public void setSlideTargetPosVer(int value){
        if (slideTargetPos >= 0 && slideTargetPos <= robotConstants.slideMaxVerLimit){
            slideTargetPos = value;
        }
    }

    public void setSlideTargetPosHor(int value){
        if (slideTargetPos >= 0 && slideTargetPos <= robotConstants.slideMaxHorLimit){
            slideTargetPos = value;
        } else if (slideTargetPos >= robotConstants.slideMaxHorLimit) {
            slideTargetPos = (int) robotConstants.slideMaxHorLimit;
        }
    }

    public void setSlidePIDController(PIDController controller){
        slidePIDController = controller;
    }
    public void setHangOn(boolean joe) {
        hangOn=joe;
    }
}
