package frc.robot.commands.auto;

import frc.robot.commands.drivecommands.SimpleDrive;

public class DriveFoward extends AutoCommand{
    public DriveFoward(){
        super(new SimpleDrive(10.0, 0.0, 0.0).withTimeout(5));
    }
}
