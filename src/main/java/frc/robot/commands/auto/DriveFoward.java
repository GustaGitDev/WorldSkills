package frc.robot.commands.auto;

import frc.robot.commands.drivecommands.SimpleDrive;

public class DriveFoward extends AutoCommand{
    public DriveFoward(){
        super(new SimpleDrive(0.0, 0.5, 0.0).withTimeout(5));
    }
}
