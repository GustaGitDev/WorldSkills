package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.OMS;

public class SetClawDeg extends InstantCommand {
    public SetClawDeg(OMS oms, double deg) {
        super(() -> oms.setClawDeg(deg));
    }
}
