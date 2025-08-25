package frc.robot.commands.drivecommands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveBase;

public class DriveWithPid extends CommandBase{
    private static final DriveBase drive = RobotContainer.drivebase;

    private double setpointDistance;
    private double setpointYaw;

    PIDController pidYaxis;
    PIDController pidZaxis;

    public DriveWithPid(double setpointDistance, double epsilonDistance, double setpointYaw, double epsilonYaw)
    {
       this.setpointDistance = setpointDistance;
       this.setpointYaw = setpointYaw;
        addRequirements(drive);

        pidYaxis = new PIDController(1, 0, 0);
        pidYaxis.setTolerance(epsilonDistance);

        pidZaxis = new PIDController(0.1, 0, 0);
        pidZaxis.setTolerance(epsilonYaw);
    }
    @Override
    public void initialize() {
        drive.resetEncoders();
        drive.resetYaw();
        pidYaxis.reset();
        pidZaxis.reset();
    }
    @Override
    public void execute()
    {
        drive.holonomicDrive(0.0,
        MathUtil.clamp(pidYaxis.calculate(drive.getAverageForwardEncoderDistance(), setpointDistance),-0.5, 0.5),
        MathUtil.clamp(pidZaxis.calculate(drive.getYaw(), setpointYaw), -1, 1));
    }
    @Override
    public void end(boolean interrupted)
    {
        drive.setDriveMotorSpeeds(0.0, 0.0, 0.0);
    }
    @Override
    public boolean isFinished()
    {
            return pidYaxis.atSetpoint();
    }
}