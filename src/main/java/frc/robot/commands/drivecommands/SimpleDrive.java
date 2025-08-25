package frc.robot.commands.drivecommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveBase;

public class SimpleDrive extends CommandBase{
    private static final DriveBase drive = RobotContainer.drivebase;

    private double x;
    private double y;
    private double z;

    public SimpleDrive(Double x, Double y, Double z)
    {
        this.x = x;
        this.y = y;
        this.z = z;
        addRequirements(drive);
    }
    @Override
    public void execute()
    {
        drive.holonomicDrive(x, y, z);
    }
    @Override
    public void end(boolean interrupted)
    {
        drive.setDriveMotorSpeeds(0.0, 0.0, 0.0);
    }
    @Override
    public boolean isFinished()
    {
            return false;
    }
}