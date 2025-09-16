package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveBase;

public class AutonomoTESTE extends SequentialCommandGroup {

  private static final DriveBase drive = RobotContainer.drivebase;

  public AutonomoTESTE() {
    addCommands(
        // 1) Zera sensores exatamente no momento em que o autônomo começa
        new InstantCommand(() -> {
          drive.resetEncoders();
          drive.resetYaw();
        }, drive),
        // (Opcional) pequena espera para o NavX "assentar" a leitura de yaw
        new WaitCommand(0.1),

        // 2) Anda reto mantendo 0°
        new DriveStraightWithPid(30.0, 0.0, null, 20),

        // 3) Gira a 90° e dá uma pausa
        new RotateToAngleWithPid(90.0, null, 10), new WaitCommand(1.0)


    );
  }
}
