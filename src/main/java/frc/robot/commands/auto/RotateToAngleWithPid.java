package frc.robot.commands.auto;

import frc.robot.commands.drivecommands.DriveWithPid;

/**
 * Gira no lugar até atingir o ângulo absoluto especificado (graus).
 * 
 * @param angleDeg   Heading absoluto alvo (graus).
 * @param yawMult    Multiplicador da correção de yaw. Se null, usa o slider do dashboard.
 */
public class RotateToAngleWithPid extends AutoCommand {
    //public RotateToAngleWithPid(double angleDeg, Double yawMult) {
     //   super(new DriveWithPid(0.0, angleDeg, yawMult));
   // }

    public RotateToAngleWithPid(double angleDeg, Double yawMult, double timeoutSeconds) {
        super(new DriveWithPid(0.0, angleDeg, yawMult).withTimeout(timeoutSeconds));
    }
}