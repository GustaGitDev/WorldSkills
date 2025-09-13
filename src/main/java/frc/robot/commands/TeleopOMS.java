package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.gamepad.OI;
import frc.robot.subsystems.OMS;

public class TeleopOMS extends CommandBase {
    private static final OMS oms = RobotContainer.oms;
    private static final OI  oi  = RobotContainer.oi;

    // ===== Setpoints iniciais =====
    private double linearDeg = 100.0; // linear
    private double rotDeg    = 0.0;   // rotação da garra

    // ===== Parâmetros =====
    private static final double STEP_DEG_LINEAR  = 2.0;   // degrau do linear
    private static final double STEP_INTERVAL_MS = 60.0;  // intervalo entre passos (ms)
    private static final double STEP_DEG_ROT     = 3.0;   // degrau da rotação (nudge simples)

    // timers p/ repetição do linear
    private double nextLinearStepAtMs = 0.0;

    public TeleopOMS() { addRequirements(oms); }

    @Override
    public void initialize() {
        oms.setLinearGarraDeg(linearDeg);
        oms.setGarraRotDeg(rotDeg);
        oms.setClawDeg(60.0); // fechado
        oms.resetEncoders();

        double nowMs = Timer.getFPGATimestamp() * 1000.0;
        nextLinearStepAtMs = nowMs;
    }

    @Override
    public void execute() {
        double nowMs = Timer.getFPGATimestamp() * 1000.0;

        // botões
        boolean elevUp   = oi.getDriveRightBumper();
        boolean elevDown = oi.getDriveRightTrigger();
        boolean clawOpen = oi.getDriveLeftBumper();

        boolean dpadUp    = oi.getDriveDPadUp();
        boolean dpadDown  = oi.getDriveDPadDown();
        boolean dpadLeft  = oi.getDriveDPadLeft();
        boolean dpadRight = oi.getDriveDPadRight();

        // ===== Elevator =====
        if (elevUp)        oms.setElevatorMotorSpeed(0.7);
        else if (elevDown) oms.setElevatorMotorSpeed(-0.7);
        else               oms.setElevatorMotorSpeed(0.0);

        // ===== Garra abre/fecha =====
        oms.setClawDeg(clawOpen ? 120.0 : 60.0);

        // ===== Linear: degrau repetido enquanto segura (↑/↓) =====
        if ((dpadUp || dpadDown)) {
            if (nowMs >= nextLinearStepAtMs) {
                double step = dpadUp ? +STEP_DEG_LINEAR : -STEP_DEG_LINEAR;
                linearDeg += step;
                oms.setLinearGarraDeg(linearDeg);
                nextLinearStepAtMs = nowMs + STEP_INTERVAL_MS;
            }
        } else {
            nextLinearStepAtMs = nowMs; // reseta p/ próximo pressionar
        }

        // ===== Rotação: nudges diretos (sem repetição) =====
        if (dpadLeft && !dpadRight)  rotDeg -= STEP_DEG_ROT;
        if (dpadRight && !dpadLeft)  rotDeg += STEP_DEG_ROT;
        oms.setGarraRotDeg(rotDeg);
    }

    @Override
    public void end(boolean interrupted) {
        oms.setElevatorMotorSpeed(0.0);
    }
}
