package common.commands.arm.armClawCommands;

import com.arcrobotics.ftclib.command.InstantCommand;

//import org.firstinspires.ftc.teamcodeworlds.common.robot.robotHardware;
import common.robot.robotHardware;
import common.subsystems.armSubsystem;
public class armClawGrabPos extends InstantCommand {
    public armClawGrabPos(){
        super(() -> robotHardware.getInstance().armSub.updateClawState(armSubsystem.armClawState.armClawGrabPos));
    }
}
