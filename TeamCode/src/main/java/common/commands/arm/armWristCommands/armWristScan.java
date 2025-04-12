package common.commands.arm.armWristCommands;

import com.arcrobotics.ftclib.command.InstantCommand;

//import org.firstinspires.ftc.teamcode.common.robot.robotHardwareTset;
//import org.firstinspires.ftc.teamcodeworlds.common.robot.robotHardware;
import common.robot.robotHardware;

public class  armWristScan extends InstantCommand {
    public armWristScan(double value){
        super(() -> robotHardware.getInstance().armSub.changearmWristServoPos(value));
    }
}
