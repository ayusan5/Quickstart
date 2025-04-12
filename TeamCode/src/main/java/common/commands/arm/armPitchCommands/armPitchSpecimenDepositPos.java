package common.commands.arm.armPitchCommands;

import com.arcrobotics.ftclib.command.InstantCommand;

import common.robot.robotHardware;
import common.subsystems.armSubsystem;

public class armPitchSpecimenDepositPos extends InstantCommand {
    public armPitchSpecimenDepositPos(){
        super(() -> robotHardware.getInstance().armSub.updatePitchState((armSubsystem.armPitchState.armPitchSpecimenDepositPos)));
    }
}
