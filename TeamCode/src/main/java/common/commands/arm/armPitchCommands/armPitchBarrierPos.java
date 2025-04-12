package common.commands.arm.armPitchCommands;

import com.arcrobotics.ftclib.command.InstantCommand;

import common.robot.robotHardware;
import common.subsystems.armSubsystem;

public class armPitchBarrierPos extends InstantCommand {
    public armPitchBarrierPos(){
        super(() -> robotHardware.getInstance().armSub.updatePitchState((armSubsystem.armPitchState.armPitchBarrierPos)));
    }
}