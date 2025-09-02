package frc.robot.commands.auto;

import frc.robot.commands.drivecommands.DriveWithPid;

public class Rotate180WithPid extends AutoCommand {
    public Rotate180WithPid() {
        // Sem avanço (0.0 m), trava heading em 180°, multiplicador de yaw vindo do dashboard
        super(new DriveWithPid(0.0, 180.0, null).withTimeout(10));
    }
}
