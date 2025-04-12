package common.commands.slide;

import com.arcrobotics.ftclib.command.InstantCommand;

import common.robot.robotHardware;

public class setHang extends InstantCommand {
    public setHang(boolean joe){
        super(() -> robotHardware.getInstance().slideSub.setHangOn(joe));
    }
}