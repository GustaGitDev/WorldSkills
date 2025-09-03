package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveBase;

public class LabirintoA extends SequentialCommandGroup {

  private static final DriveBase drive = RobotContainer.drivebase;

  public LabirintoA() {
    double stopDist = 0.3;  // parar a 60 cm
    double maxStep  = 300.00;  // fail-safe de 3 m por trecho
    double pauseS   = 0.25;  // pausas curtas

    addCommands(
      new InstantCommand(() -> { drive.resetEncoders(); drive.resetYaw(); }, drive),
      new WaitCommand(0.15),

      new DriveUntilWallWithPid(stopDist,   0.0,  maxStep),
      new WaitCommand(pauseS),
      new RotateToAngleWithPid( 90.0, null, 3.0),
      new WaitCommand(pauseS),

      new DriveUntilWallWithPid(stopDist,  90.0,  maxStep),
      new WaitCommand(pauseS),
      new RotateToAngleWithPid(180.0, null, 3.0),
      new WaitCommand(pauseS),

      new DriveUntilWallWithPid(stopDist, 180.0,  maxStep),
      new WaitCommand(pauseS),
      new RotateToAngleWithPid(-90.0, null, 3.0),
      new WaitCommand(pauseS),

      new DriveUntilWallWithPid(stopDist, -90.0,  maxStep),
      new WaitCommand(pauseS)
    );
  }
}
