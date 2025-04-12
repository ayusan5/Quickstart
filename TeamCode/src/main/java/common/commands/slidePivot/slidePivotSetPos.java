package common.commands.slidePivot;

import com.arcrobotics.ftclib.command.InstantCommand;

import common.robot.robotHardware;

public class slidePivotSetPos extends InstantCommand {
    public slidePivotSetPos(int slidepivottargetval){
        super(() -> robotHardware.getInstance().slidePivotSub.setSlidePivotTargetPos(slidepivottargetval));
    }
}