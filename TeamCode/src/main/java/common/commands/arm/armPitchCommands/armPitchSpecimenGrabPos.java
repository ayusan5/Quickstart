package common.commands.arm.armPitchCommands;

import com.arcrobotics.ftclib.command.InstantCommand;

import common.robot.robotHardware;
import common.subsystems.armSubsystem;

public class armPitchSpecimenGrabPos extends InstantCommand {
    public armPitchSpecimenGrabPos(){
        super(() -> robotHardware.getInstance().armSub.updatePitchState((armSubsystem.armPitchState.armPitchSpecimenGrabPos)));
    }
}
