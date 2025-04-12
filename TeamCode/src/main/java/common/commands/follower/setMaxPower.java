package common.commands.follower;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.pedropathing.pathgen.Path;

import common.robot.robotHardware;

public class setMaxPower extends InstantCommand {
    public setMaxPower(double power){
        super(() -> robotHardware.getInstance().follower.setMaxPower(power));
    }
}