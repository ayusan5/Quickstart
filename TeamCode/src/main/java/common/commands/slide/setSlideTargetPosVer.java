package common.commands.slide;

import com.arcrobotics.ftclib.command.InstantCommand;

import common.robot.robotHardware;

public class setSlideTargetPosVer extends InstantCommand {
    public setSlideTargetPosVer(int slidetargetval){
        super(() -> robotHardware.getInstance().slideSub.setSlideTargetPosVer(slidetargetval));
    }
}