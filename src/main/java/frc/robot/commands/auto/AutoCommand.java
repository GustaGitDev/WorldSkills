package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * Base simples para sequências de autônomo.
 */
public abstract class AutoCommand extends SequentialCommandGroup {
  public AutoCommand() { super(); }
  public AutoCommand(Command... cmd) { super(cmd); }
}
