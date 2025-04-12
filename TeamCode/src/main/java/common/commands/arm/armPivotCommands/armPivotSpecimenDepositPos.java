package common.commands.arm.armPivotCommands;

import com.arcrobotics.ftclib.command.InstantCommand;

import common.robot.robotHardware;
import common.subsystems.armSubsystem;

public class armPivotSpecimenDepositPos extends InstantCommand {
    public armPivotSpecimenDepositPos(){
        super(() -> robotHardware.getInstance().armSub.updatePivotState((armSubsystem.armPivotState.armPivotSpecimenDepositPos)));
    }
}
