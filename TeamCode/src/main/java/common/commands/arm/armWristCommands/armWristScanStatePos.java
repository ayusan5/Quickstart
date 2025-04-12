package common.commands.arm.armWristCommands;

import com.arcrobotics.ftclib.command.InstantCommand;

import common.robot.robotHardware;
import common.subsystems.armSubsystem;

public class armWristScanStatePos extends InstantCommand {
    public armWristScanStatePos(){
        super(() -> robotHardware.getInstance().armSub.updateWristState((armSubsystem.armWristState.armWristScanState)));
    }
}
