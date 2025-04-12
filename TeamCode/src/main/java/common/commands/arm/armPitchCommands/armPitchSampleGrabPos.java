package common.commands.arm.armPitchCommands;

import com.arcrobotics.ftclib.command.InstantCommand;

import common.robot.robotHardware;
import common.subsystems.armSubsystem;

public class armPitchSampleGrabPos extends InstantCommand {
    public armPitchSampleGrabPos(){
        super(() -> robotHardware.getInstance().armSub.updatePitchState((armSubsystem.armPitchState.armPitchSampleGrabPos)));
    }
}
