package common.commands.arm.armWristCommands;

import com.arcrobotics.ftclib.command.InstantCommand;

import common.robot.robotHardware;
import common.subsystems.armSubsystem;

public class armWristCenterPos extends InstantCommand {
    public armWristCenterPos(){
        super(() -> robotHardware.getInstance().armSub.updateWristState((armSubsystem.armWristState.armWristCenterPos)));
    }
}
