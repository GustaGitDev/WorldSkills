package frc.robot.commands.auto;

import frc.robot.commands.drivecommands.DriveWithPid;

public class DriveFowardWithPid extends AutoCommand {
    public DriveFowardWithPid() {
        // 1.0 m adiante, heading lock no ângulo atual, timeout de 5 s
        super(new DriveWithPid(100.0).withTimeout(5));
    }
}
