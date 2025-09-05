package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.gamepad.OI;
import frc.robot.subsystems.OMS;

public class TeleopOMS extends CommandBase {
    private static final OMS oms = RobotContainer.oms;
    private static final OI oi = RobotContainer.oi;

    boolean up;
    boolean down;
    boolean open;

    public TeleopOMS() {
        addRequirements(oms);
    }

    @Override
    public void initialize() {
        oms.resetEncoders();
    }

    @Override
    public void execute() {
        up = oi.getDriveRightBumper();
        down = oi.getDriveRightTrigger();
        open = oi.getDriveLeftBumper();

        if (up) {
            oms.setElevatorMotorSpeed(0.7);
        } else if (down) {
            oms.setElevatorMotorSpeed(-0.7);
        } else {
            oms.setElevatorMotorSpeed(0);
        }
        if (open) {
            oms.setServoPosition(150);
        } else {
            oms.setServoPosition(0.0);
        }

    }
}