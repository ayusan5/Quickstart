package common.commands.arm.armPivotCommands;

import com.arcrobotics.ftclib.command.InstantCommand;

import common.robot.robotHardware;
import common.subsystems.armSubsystem;

public class armPivotBarrierPos extends InstantCommand {
    public armPivotBarrierPos(){
        super(() -> robotHardware.getInstance().armSub.updatePivotState((armSubsystem.armPivotState.armPivotBarrierPos)));
    }
}
