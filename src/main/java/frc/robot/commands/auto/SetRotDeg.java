package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.OMS;

public class SetRotDeg extends InstantCommand {
    public SetRotDeg(OMS oms, double deg) {
        super(() -> oms.setGarraRotDeg(deg));
    }
}
