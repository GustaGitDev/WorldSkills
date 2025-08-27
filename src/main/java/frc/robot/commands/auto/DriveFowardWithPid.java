package frc.robot.commands.auto;

import frc.robot.commands.drivecommands.DriveWithPid;

public class DriveFowardWithPid extends AutoCommand{
    public DriveFowardWithPid(){
        super(new DriveWithPid(1000, 10, 0.0, 1.0,0.2).withTimeout(5));
    }
}
