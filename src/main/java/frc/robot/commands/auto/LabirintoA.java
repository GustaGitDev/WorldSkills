package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveBase;

/**
 * Sequência de teste de labirinto. Usa o comando ScanAndChooseDirection para
 * decidir automaticamente para qual lado seguir baseado nas leituras da câmera
 * de profundidade.
 */
public class LabirintoA extends SequentialCommandGroup {

  private static final DriveBase drive = RobotContainer.driveBase;

  public LabirintoA() {
    double stopDist = 0.27; // parar a 25 cm da parede
    double maxStep = 300.00; // fail-safe de 5 m
    double pauseS = 0.25; // pausas curtas (settle time)

    addCommands(
        // Zera encoders e yaw antes de começar
        new InstantCommand(() -> {
          drive.resetEncoders();
          drive.resetYaw();
        }, drive),

        // Usa o comando de varredura e decisão
        new DriveUntilWallWithPid(stopDist, 0.0, maxStep), new ScanAndChooseDirection(stopDist, maxStep, pauseS),

        new InstantCommand(() -> {
          drive.resetEncoders();
          drive.resetYaw();
        }, drive),

        new DriveUntilWallWithPid(stopDist, 0.0, maxStep), new ScanAndChooseDirection(stopDist, maxStep, pauseS),

        new InstantCommand(() -> {
          drive.resetEncoders();
          drive.resetYaw();
        }, drive),

        new DriveUntilWallWithPid(stopDist, 0.0, maxStep), new ScanAndChooseDirection(stopDist, maxStep, pauseS),

        new InstantCommand(() -> {
          drive.resetEncoders();
          drive.resetYaw();
        }, drive),

        new DriveUntilWallWithPid(stopDist, 0.0, maxStep), new ScanAndChooseDirection(stopDist, maxStep, pauseS),

        new InstantCommand(() -> {
          drive.resetEncoders();
          drive.resetYaw();
        }, drive)

    );
  }
}
