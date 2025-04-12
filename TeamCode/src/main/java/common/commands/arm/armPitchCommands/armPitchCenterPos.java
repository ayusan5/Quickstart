package common.commands.arm.armPitchCommands;

import com.arcrobotics.ftclib.command.InstantCommand;

import common.robot.robotHardware;
import common.subsystems.armSubsystem;

public class armPitchCenterPos extends InstantCommand {
    public armPitchCenterPos(){
        super(() -> robotHardware.getInstance().armSub.updatePitchState((armSubsystem.armPitchState.armPitchCenterPos)));
    }
}
