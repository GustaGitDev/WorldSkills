package frc.robot.commands.drivecommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveBase;

/**
 * Reta com heading lock: - PID de distância (DriveBase) para avançar até
 * "distanceMeters" - PID de yaw (DriveBase) para manter o heading alvo Usa
 * rampas (slew) e desliga heurísticas do mixer no modo auton.
 */
public class DriveWithPid extends CommandBase {
    private static final DriveBase drive = RobotContainer.drivebase;

    private final Double distanceMetersTarget;
    // delta desejado (m). 0 = só segurar heading
    private final Double fixedYawDeg; // se null, usa yaw atual como alvo
    private final Double customYawSpeedMult; // multiplicador opcional p/ correção de yaw

    private double startDistance;

    /** Segue "distanceMeters" e trava o heading no YAW ATUAL. */
    public DriveWithPid(double distanceMeters) {
        this(distanceMeters, null, null);
    }

    /**
     * @param distanceMeters     distância a avançar (m). Se 0, não avança; apenas
     *                           segura heading.
     * @param holdYawDeg         heading absoluto alvo (graus). Se null, usa yaw
     *                           atual no initialize().
     * @param yawSpeedMultiplier multiplicador opcional p/ rapidez da correção de
     *                           yaw (null usa o do dashboard)
     */
    public DriveWithPid(double distanceMeters, Double holdYawDeg, Double yawSpeedMultiplier) {
        this.distanceMetersTarget = distanceMeters * 4.6;
        this.fixedYawDeg = holdYawDeg;
        this.customYawSpeedMult = yawSpeedMultiplier;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        drive.resetPids();
        drive.setAutoClosedLoop(true); // mixer "puro" no auton

        // ponto de partida (reta em Y)
        startDistance = drive.getForwardDistance();

        // yaw alvo
        double targetYaw = (fixedYawDeg != null) ? fixedYawDeg : drive.getYaw();
        drive.setYawTargetDeg(targetYaw);

        // setpoint SEM inversão (sempre start + distância desejada)
        if (distanceMetersTarget != 0.0) {
            drive.setDistanceTargetMeters(startDistance + distanceMetersTarget);
        } else {
            drive.setDistanceTargetMeters(0.0);
        }

        if (customYawSpeedMult != null) {
            drive.setYawSpeedMultiplier(customYawSpeedMult);
        }
    }

    @Override
    public void execute() {

        double y = drive.computeDistancePidOutput(); // reta (Y)
        double z = drive.computeYawPidOutput(); // heading

        // rampas
        y = drive.applySlewY(y);
        z = drive.applySlewZ(z);

        // aplique o referencial do HARDWARE no comando (sempre aqui)
        if (drive.shouldInvertY())
            y = -y;

        drive.holonomicDrive(0.0, y, z);
    }

    @Override
    public void end(boolean interrupted) {
        drive.setAutoClosedLoop(false); // volta heurísticas para teleop
        drive.setDriveMotorSpeeds(0.0, 0.0, 0.0);
    }

    @Override
    public boolean isFinished() {
        return drive.atYawSetpoint() && drive.atDistanceSetpoint();
    }
}
