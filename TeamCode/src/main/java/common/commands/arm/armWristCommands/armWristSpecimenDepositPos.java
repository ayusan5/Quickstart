package common.commands.arm.armWristCommands;

import com.arcrobotics.ftclib.command.InstantCommand;

import common.robot.robotHardware;
import common.subsystems.armSubsystem;

public class armWristSpecimenDepositPos extends InstantCommand {
    public armWristSpecimenDepositPos(){
        super(() -> robotHardware.getInstance().armSub.updateWristState((armSubsystem.armWristState.armWristSpecimenDepositPos)));
    }
}
