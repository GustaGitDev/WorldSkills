package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.OMS;

public class SetLinearDeg extends InstantCommand {
    public SetLinearDeg(OMS oms, double deg) {
        super(() -> oms.setLinearGarraDeg(deg));
    }
}
