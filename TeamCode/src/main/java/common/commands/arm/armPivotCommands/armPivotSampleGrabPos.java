package common.commands.arm.armPivotCommands;

import com.arcrobotics.ftclib.command.InstantCommand;

import common.robot.robotHardware;
import common.subsystems.armSubsystem;

public class armPivotSampleGrabPos extends InstantCommand {
    public armPivotSampleGrabPos(){
        super(() -> robotHardware.getInstance().armSub.updatePivotState((armSubsystem.armPivotState.armPivotSampleGrabPos)));
    }
}
