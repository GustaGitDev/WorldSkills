package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.OMS;

public class MoveElevatorTo extends CommandBase {
    private final OMS oms;
    private final double target;
    private final boolean keepPosMode;

    public MoveElevatorTo(OMS oms, double target) {
        this(oms, target, false);
    }

    public MoveElevatorTo(OMS oms, double target, boolean keepPosMode) {
        this.oms = oms;
        this.target = target;
        this.keepPosMode = keepPosMode;
        addRequirements(oms);
    }

    @Override
    public void initialize() {
        oms.enableElevatorPositionMode(true);
        oms.setElevatorTarget(target);
    }

    @Override
    public boolean isFinished() {
        return oms.atElevatorTarget();
    }

    @Override
    public void end(boolean interrupted) {
        if (!keepPosMode) {
            oms.enableElevatorPositionMode(false);
            oms.setElevatorMotorSpeed(0.0);
        }
    }
}
