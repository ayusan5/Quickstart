package common.commands.arm.armPivotCommands;

import com.arcrobotics.ftclib.command.InstantCommand;

import common.robot.robotHardware;
import common.subsystems.armSubsystem;

public class armPivotSpecimenGrabPos extends InstantCommand {
    public armPivotSpecimenGrabPos(){
        super(() -> robotHardware.getInstance().armSub.updatePivotState((armSubsystem.armPivotState.armPivotSpecimenGrabPos)));
    }
}
