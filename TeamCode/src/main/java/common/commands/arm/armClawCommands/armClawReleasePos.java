package common.commands.arm.armClawCommands;

import com.arcrobotics.ftclib.command.InstantCommand;

import common.robot.robotHardware;
import common.subsystems.armSubsystem;

public class armClawReleasePos extends InstantCommand {
    public armClawReleasePos(){
        super(() -> robotHardware.getInstance().armSub.updateClawState(armSubsystem.armClawState.armClawReleasePos));
    }
}
