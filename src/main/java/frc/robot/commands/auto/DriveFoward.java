package frc.robot.commands.auto;

import frc.robot.commands.drivecommands.SimpleDrive;

/**
 * Anda para frente por 2.0s e para.
 * Mantive o nome "Foward" para combinar com padroes comuns em repos similares.
 */
public class DriveFoward extends AutoCommand {
  public DriveFoward() {
    // x, y, z = (strafe, forward, rotate). Ajuste o 0.5 se quiser mais/menos velocidade.
    super(new SimpleDrive(0.0, 0.5, 0.0).withTimeout(2.0));
  }
}
