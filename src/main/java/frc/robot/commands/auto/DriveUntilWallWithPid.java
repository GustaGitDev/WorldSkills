package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.DepthWallRange;

/**
 * Avança em linha reta com heading lock até a parede ficar a <= stopDistanceMeters.
 * Usa PID de distância APENAS como "trilho" (fail-safe: maxTravelMeters), e PID de yaw para heading.
 */
public class DriveUntilWallWithPid extends CommandBase {
    private static final DriveBase drive = RobotContainer.drivebase;
    private static final DepthWallRange wallRange = RobotContainer.wallRange;

    private final double stopDistanceMeters;     // distância de parada em relação à parede (ex.: 0.60 m)
    private final Double holdYawDeg;             // heading alvo absoluto; se null, usa yaw atual
    private final double maxTravelMeters;        // percurso máximo (fail-safe), em metros
    private final Double customYawSpeedMult;     // multiplicador opcional para correção de yaw

    private double startForward;
    private double maxTravelInternalUnits;       // maxTravelMeters convertido pela calibração (×4.6)
    private boolean finished = false;

    /** Construtor básico, sem custom yaw speed multiplier. */
    public DriveUntilWallWithPid(double stopDistanceMeters, Double holdYawDeg, double maxTravelMeters) {
        this(stopDistanceMeters, holdYawDeg, maxTravelMeters, null);
    }

    /** Construtor com multiplicador opcional de yaw (segue padrão do DriveWithPid). */
    public DriveUntilWallWithPid(double stopDistanceMeters, Double holdYawDeg, double maxTravelMeters, Double yawSpeedMultiplier) {
        this.stopDistanceMeters = stopDistanceMeters;
        this.holdYawDeg = holdYawDeg;
        this.maxTravelMeters = maxTravelMeters;
        this.customYawSpeedMult = yawSpeedMultiplier;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        finished = false;

        drive.resetPids();
        drive.setAutoClosedLoop(true);

        startForward = drive.getForwardDistance();

        // Travar heading: se não for passado, usa o yaw atual (consistente com DriveWithPid)
        double targetYaw = (holdYawDeg != null) ? holdYawDeg : drive.getYaw();
        drive.setYawTargetDeg(targetYaw);

        // Calibração igual ao DriveWithPid (metros -> unidades internas)
        maxTravelInternalUnits = maxTravelMeters * 4.6;

        // O setpoint de distância é "longo": apenas um trilho/limite. Parada real vem do sensor da parede.
        drive.setDistanceTargetMeters(startForward + maxTravelInternalUnits);

        if (customYawSpeedMult != null) {
            drive.setYawSpeedMultiplier(customYawSpeedMult);
        }
    }

    @Override
    public void execute() {
        // Atualiza filtro/estimativa do sensor de parede
        wallRange.update();

        // PIDs
        double y = drive.computeDistancePidOutput(); // avanço em Y (reta)
        double z = drive.computeYawPidOutput();      // correção de heading

        // Rampas
        y = drive.applySlewY(y);
        z = drive.applySlewZ(z);

        // Referencial do hardware
        if (drive.shouldInvertY()) y = -y;

        // Comando de movimento
        drive.holonomicDrive(0.0, y, z);

        // Critério de parada por parede
        if (wallRange.hasValidReading()) {
            double dist = wallRange.getFilteredDistanceMeters();
            if (dist <= stopDistanceMeters) {
                finished = true;
                return;
            }
        }

        // Fail-safe por percurso máximo
        double traveled = Math.abs(drive.getForwardDistance() - startForward);
        if (traveled >= maxTravelInternalUnits) {
            finished = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        drive.setAutoClosedLoop(false);
        drive.setDriveMotorSpeeds(0.0, 0.0, 0.0);
    }

    @Override
    public boolean isFinished() {
        // Não esperamos o setpoint de distância; quem manda é a parede ou o fail-safe
        return finished;
    }
}
