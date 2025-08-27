package frc.robot.commands.auto;

import frc.robot.commands.drivecommands.DriveWithPid;

public class Rotate180WithPid extends AutoCommand {
    public Rotate180WithPid() {
        // Configura o setpointYaw para 180 graus e uma tolerância (epsilonYaw)
        super(new DriveWithPid(0.0, 10.0, 180.0, 1.0, 0.08).withTimeout(5));  // 180 graus, tolerância de 1 graus
    }
}
