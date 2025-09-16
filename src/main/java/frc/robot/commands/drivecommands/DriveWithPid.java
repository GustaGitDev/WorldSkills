package frc.robot.commands.drivecommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveBase;

/**
 * Reta com heading lock:
 * - PID de distância (DriveBase) para avançar até "distanceMeters"
 * - PID de heading (DriveBase) para manter/atingir o ângulo alvo
 * Usa rampas (slew) e desliga heurísticas do mixer no modo auton.
 */
public class DriveWithPid extends CommandBase {
    private static final DriveBase drive = RobotContainer.drivebase;

    private final Double distanceMetersTarget;   // delta desejado (m). 0 = só segurar heading
    private final Double fixedYawDeg;            // heading absoluto alvo (graus). Se null, “trava” o heading atual (zera integrador e mira 0°)
    private final Double customYawSpeedMult;     // multiplicador opcional p/ correção de heading

    private double startDistance;

    /** Segue "distanceMeters" e trava o heading no HEADING ATUAL (via zero do integrador). */
    public DriveWithPid(double distanceMeters) {
        this(distanceMeters, null, null);
    }

    /**
     * @param distanceMeters     distância a avançar (m). Se 0, não avança; apenas segura heading.
     * @param holdYawDeg         heading absoluto alvo (graus). Se null, “trava” o heading atual (zera integrador e usa 0°).
     * @param yawSpeedMultiplier multiplicador opcional p/ rapidez da correção de heading (null usa o do dashboard)
     */
    public DriveWithPid(double distanceMeters, Double holdYawDeg, Double yawSpeedMultiplier) {
        // mantém seu fator 4.6, caso esteja convertendo “unidade interna” -> metros de fato
        this.distanceMetersTarget = distanceMeters * 4.6;
        this.fixedYawDeg = holdYawDeg;
        this.customYawSpeedMult = yawSpeedMultiplier;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        // Reset de PIDs e rampas do DriveBase
        drive.resetPids();

        // Mixer “puro” no auton (sem heurísticas de teleop)
        drive.setAutoClosedLoop(true);

        // Ponto de partida (reta em Y) para alvo de distância
        startDistance = drive.getForwardDistance();

        // Heading alvo:
        if (fixedYawDeg == null) {
            // “Travar” o heading atual: zera o integrador e mira 0°
            drive.zeroIntegratedHeading();
            drive.setYawTargetDeg(0.0);
        } else {
            // Heading absoluto alvo
            drive.setYawTargetDeg(fixedYawDeg);
        }

        // Setpoint de distância (sempre absoluto: posição atual + delta)
        if (distanceMetersTarget != 0.0) {
            drive.setDistanceTargetMeters(startDistance + distanceMetersTarget);
        } else {
            // 0 => não avançar; DriveBase já trata computeDistancePidOutput()=0 quando SP=0
            drive.setDistanceTargetMeters(0.0);
        }

        // Tolerância do PID de heading (padrão 1.0°; ajuste se quiser)
        drive.pidZaxis.setTolerance(1.0);

        // Multiplicador de velocidade do heading (se informado)
        if (customYawSpeedMult != null) {
            drive.setYawSpeedMultiplier(customYawSpeedMult);
        }
    }

    @Override
    public void execute() {
        // Saídas PID (DriveBase já usa heading integrado + clamp/deadband sensatos)
        double y = drive.computeDistancePidOutput(); // reta (Y)
        double z = drive.computeYawPidOutput();      // heading

        // Opcional: kick estático para vencer atrito (comente se não precisar)
        z = addStaticKick(z, 0.08);

        // Rampas
        y = drive.applySlewY(y);
        z = drive.applySlewZ(z);

        // Referencial do hardware aplicado no comando (mantido do seu código)
        if (drive.shouldInvertY()) y = -y;

        // Dirige (x=0 para não “andar de lado” enquanto faz reta+giro)
        drive.holonomicDrive(0.0, y, z);
    }

    @Override
    public void end(boolean interrupted) {
        // Volta heurísticas de teleop e para motores
        drive.setAutoClosedLoop(false);
        drive.setDriveMotorSpeeds(0.0, 0.0, 0.0);
    }

    @Override
    public boolean isFinished() {
        // Termina quando chegou no heading E na distância (ou use timeout externamente)
        return drive.atYawSetpoint() && drive.atDistanceSetpoint();
    }

    /** “Empurrãozinho” estático simétrico para sair do lugar. */
    private static double addStaticKick(double out, double kMin) {
        if (Math.abs(out) < 1e-6) return 0.0;
        double sign = Math.signum(out);
        double mag  = Math.abs(out);
        if (mag < kMin) mag = kMin;
        return sign * mag;
    }
}
