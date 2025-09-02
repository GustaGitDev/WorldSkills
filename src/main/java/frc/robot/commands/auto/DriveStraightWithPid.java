package frc.robot.commands.auto;

import frc.robot.commands.drivecommands.DriveWithPid;

/**
 * Anda em linha reta por uma distância em metros.
 * 
 * @param meters     Distância em metros a percorrer.
 * @param angleDeg   Heading absoluto a manter (graus). Se null, usa o heading atual.
 * @param yawMult    Multiplicador da correção de yaw. Se null, usa o slider do dashboard.
 */
public class DriveStraightWithPid extends AutoCommand {
    public DriveStraightWithPid(double meters, Double angleDeg, Double yawMult) {
        super(new DriveWithPid(meters, angleDeg, yawMult));
    }

    public DriveStraightWithPid(double meters, Double angleDeg, Double yawMult, double timeoutSeconds) {
        super(new DriveWithPid(meters, angleDeg, yawMult).withTimeout(timeoutSeconds));
    }
}
