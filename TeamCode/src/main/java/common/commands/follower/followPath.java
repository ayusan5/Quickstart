package common.commands.follower;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.pedropathing.pathgen.Path;

import common.robot.robotHardware;
import common.subsystems.armSubsystem;

public class followPath extends InstantCommand {
    public followPath(Path path){
        super(() -> robotHardware.getInstance().follower.followPath(path));
    }
}