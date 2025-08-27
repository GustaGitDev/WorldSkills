package frc.robot.commands.drivecommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveBase;

/**
 * Mantem o robo dirigindo com valores constantes enquanto o comando estiver ativo.
 * Usa o mesmo subsistema de drive do teleop.
 */
public class SimpleDrive extends CommandBase {
  private static final DriveBase drive = RobotContainer.drivebase;
  private final double x;
  private final double y;
  private final double z;

  public SimpleDrive(double x, double y, double z) {
    this.x = x;
    this.y = y;
    this.z = z;
    addRequirements(drive);
  }

  @Override
  public void execute() {
    // Chama a mesma funcao usada no teleop (holonomic/mecanum)
    drive.holonomicDrive(x, y, z);
  }

  @Override
  public void end(boolean interrupted) {
    // Para tudo ao terminar
    drive.setDriveMotorSpeeds(0.0, 0.0, 0.0);
  }

  @Override
  public boolean isFinished() {
    return false; // quem limita tempo e o withTimeout na sequencia do auto
  }
}
