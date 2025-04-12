package common.commands.arm.armPivotCommands;

import com.arcrobotics.ftclib.command.InstantCommand;

import common.robot.robotHardware;
import common.subsystems.armSubsystem;

public class armPivotInitPos extends InstantCommand {
    public armPivotInitPos(){
        super(() -> robotHardware.getInstance().armSub.updatePivotState(armSubsystem.armPivotState.armPivotInitPos));
    }
}
