package common.commands.slide;

import com.arcrobotics.ftclib.command.InstantCommand;

import common.robot.robotHardware;

public class setSlideTargetPosHor extends InstantCommand {
    public setSlideTargetPosHor(int slidetargetval){
        super(() -> robotHardware.getInstance().slideSub.setSlideTargetPosHor(slidetargetval));
    }
}