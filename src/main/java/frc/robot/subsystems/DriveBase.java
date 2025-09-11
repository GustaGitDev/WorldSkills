package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.studica.frc.TitanQuad;
import com.studica.frc.TitanQuadEncoder;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpiutil.math.MathUtil;

import edu.wpi.first.wpilibj.SlewRateLimiter; // (se WPILib nova, pode usar edu.wpi.first.math.filter.SlewRateLimiter)
import edu.wpi.first.wpilibj.Timer;

import frc.robot.Constants;

public class DriveBase extends SubsystemBase {
    // === Motores/Encoders/Gyro ===
    private final TitanQuad leftMotor;
    private final TitanQuad rightMotor;
    private final TitanQuad backMotor;

    private final TitanQuadEncoder leftEncoder;
    private final TitanQuadEncoder rightEncoder;
    private final TitanQuadEncoder backEncoder;

    private final AHRS navx;

    // === Telemetria (aba existente) ===
    private final ShuffleboardTab tab = Shuffleboard.getTab("Training Robot");
    private final NetworkTableEntry leftEncoderValue = tab.add("left encoder", 0).getEntry();
    private final NetworkTableEntry rightEncoderValue = tab.add("right encoder", 0).getEntry();
    private final NetworkTableEntry backEncoderValue = tab.add("back encoder", 0).getEntry();
    private final NetworkTableEntry gyroValue = tab.add("NavX Yaw", 0).getEntry();

    // Forward distance (diagnóstico visual)
    private final NetworkTableEntry forwardDistEntry = tab.add("Forward Dist (calc)", 0.0).getEntry();

    // === Tuning PID (aba "Tuning") ===
    private final ShuffleboardTab tuningTab = Shuffleboard.getTab("Tuning");

    // Inversão de saída Y (para casar com hardware; usada no comando, não no
    // setpoint)
    private final NetworkTableEntry invertYOutput = tuningTab.add("Invert Y Output", true).getEntry();

    public boolean shouldInvertY() {
        return invertYOutput.getBoolean(true);
    }

    // Ganhos Distância (reta em Y)
    private final NetworkTableEntry kP_Y = tuningTab.add("kP_Y", 1.0).withPosition(0, 0).getEntry();
    private final NetworkTableEntry kI_Y = tuningTab.add("kI_Y", 0.0).withPosition(1, 0).getEntry();
    private final NetworkTableEntry kD_Y = tuningTab.add("kD_Y", 0.0).withPosition(2, 0).getEntry();

    // Ganhos Yaw/Rotação (Z)
    private final NetworkTableEntry kP_Z = tuningTab.add("kP_Z", 0.1).withPosition(0, 1).getEntry();
    private final NetworkTableEntry kI_Z = tuningTab.add("kI_Z", 0.0).withPosition(1, 1).getEntry();
    private final NetworkTableEntry kD_Z = tuningTab.add("kD_Z", 0.0).withPosition(2, 1).getEntry();

    // Setpoints / tolerâncias / multiplicadores
    private final NetworkTableEntry setpointDistance = tuningTab.add("Setpoint Distance (m)", 0.0).withPosition(4, 0)
            .getEntry();
    private final NetworkTableEntry setpointYaw = tuningTab.add("Setpoint Yaw (deg)", 0.0).withPosition(4, 1)
            .getEntry();
    private final NetworkTableEntry epsDistance = tuningTab.add("Tol Dist (m)", 0.05).withPosition(5, 0).getEntry();
    private final NetworkTableEntry epsYaw = tuningTab.add("Tol Yaw (deg)", 1.0).withPosition(5, 1).getEntry();
    private final NetworkTableEntry speedMultiplier = tuningTab.add("Yaw Speed Mult", 1.0).withPosition(6, 1)
            .getEntry();

    // Controladores PID
    private final PIDController pidYaxis = new PIDController(1, 0, 0); // distância (reta em Y)
    private final PIDController pidZaxis = new PIDController(0.1, 0, 0); // yaw (Z)

    // Suavização das saídas (rampa)
    private final SlewRateLimiter yLimiter = new SlewRateLimiter(1.2); // reta
    private final SlewRateLimiter zLimiter = new SlewRateLimiter(2.0); // giro

    // Modo "closed-loop" p/ auton: desliga heurísticas no mixer
    private boolean autoClosedLoop = false;

    public void setAutoClosedLoop(boolean on) {
        autoClosedLoop = on;
    }

    // Deadband utilitário
    private static double deadband(double v, double db) {
        return Math.abs(v) < db ? 0.0 : v;
    }

    // Throttle da telemetria
    private double lastTelemTime = 0.0;

    public DriveBase() {
        leftMotor = new TitanQuad(Constants.TITAN_ID, Constants.M2);
        rightMotor = new TitanQuad(Constants.TITAN_ID, Constants.M0);
        backMotor = new TitanQuad(Constants.TITAN_ID, Constants.M1);

        leftEncoder = new TitanQuadEncoder(leftMotor, Constants.M2, Constants.WHEEL_DIST_PER_TICK);
        rightEncoder = new TitanQuadEncoder(rightMotor, Constants.M0, Constants.WHEEL_DIST_PER_TICK);
        backEncoder = new TitanQuadEncoder(backMotor, Constants.M1, Constants.WHEEL_DIST_PER_TICK);

        // Ajuste reverses conforme hardware (cada chamada inverte):
        // leftEncoder.setReverseDirection();
        // rightEncoder.setReverseDirection();
        // backEncoder.setReverseDirection();

        navx = new AHRS(SPI.Port.kMXP);

        // Tolerâncias iniciais
        pidYaxis.setTolerance(epsDistance.getDouble(0.05));
        pidZaxis.setTolerance(epsYaw.getDouble(1.0));

        // Evita salto ao cruzar ±180°
        pidZaxis.enableContinuousInput(-180.0, 180.0);
    }

    // === Motores / Drive kinematics ===
    public void setLeftmotorspeed(final double speed) {
        leftMotor.set(speed);
    }

    public void setRightmotorspeed(final double speed) {
        rightMotor.set(speed);
    }

    public void setBackmotorspeed(final double speed) {
        backMotor.set(speed);
    }

    public void setDriveMotorSpeeds(final double leftSpeed, final double rightSpeed, final double backSpeed) {
        leftMotor.set(leftSpeed);
        rightMotor.set(rightSpeed);
        backMotor.set(backSpeed);
    }

    public void holonomicDrive(final double x, final double y, final double z) {
        final double yCmd = shouldInvertY() ? -y : y; 
        double rightSpeed = ((x / 3) - (yCmd / Math.sqrt(3)) + z) * Math.sqrt(3);
        double leftSpeed = ((x / 3) + (yCmd / Math.sqrt(3)) + z) * Math.sqrt(3);
        double backSpeed = (-2 * x / 3) + z;

        double max = Math.abs(rightSpeed);
        if (Math.abs(leftSpeed) > max)
            max = Math.abs(leftSpeed);
        if (Math.abs(backSpeed) > max)
            max = Math.abs(backSpeed);
        if (max > 1) {
            rightSpeed /= max;
            leftSpeed /= max;
            backSpeed /= max;
        }

        if (!autoClosedLoop) {
            // === TELEOP: heurísticas originais ===
            double leftRightSpeedDifference = Math.abs(leftSpeed - rightSpeed);
            double adjustmentFactor = 1.0 + (leftRightSpeedDifference * 1.0);
            backSpeed *= adjustmentFactor;

            double leftMotorAdjustmentFactor = 1.0 + (leftRightSpeedDifference * 0.5);
            if (Math.abs(leftSpeed) < 0.1)
                leftSpeed *= leftMotorAdjustmentFactor;

            double rightMotorAdjustmentFactor = 1.0 + (leftRightSpeedDifference * 0.5);
            if (Math.abs(rightSpeed) < 0.1)
                rightSpeed *= rightMotorAdjustmentFactor;

            leftMotor.set(leftSpeed);
            rightMotor.set(rightSpeed);
            backMotor.set(backSpeed * 1.65);
        } else {
            // === AUTÔNOMO (PID): mixer "puro", sem truques ===
            leftMotor.set(leftSpeed);
            rightMotor.set(rightSpeed);
            backMotor.set(backSpeed);
        }
    }

    // === Sensores ===
    public double getLeftEncoderDistance() {
        return leftEncoder.getEncoderDistance();
    }

    public double getRightEncoderDistance() {
        return rightEncoder.getEncoderDistance();
    }

    public double getBackEncoderDistance() {
        return backEncoder.getEncoderDistance();
    }

    public double getYaw() {
        return navx.getRoll();
    }

    public void resetEncoders() {
        leftEncoder.reset();
        rightEncoder.reset();
        backEncoder.reset();
    }

    public void resetYaw() {
        navx.zeroYaw();
    }

    /**
     * Distancia "reta" (m) — eixo Y do kiwi (back é neutro): Y = (left - right)/2
     */
    public double getForwardDistance() {
        double L = getLeftEncoderDistance();
        double R = getRightEncoderDistance();
        return (L - R) / 2.0;
    }

    // === API de Tuning/Execução de PID ===
    private void refreshPidGainsFromDashboard() {
        pidYaxis.setPID(kP_Y.getDouble(1.0), kI_Y.getDouble(0.0), kD_Y.getDouble(0.0));
        pidZaxis.setPID(kP_Z.getDouble(0.1), kI_Z.getDouble(0.0), kD_Z.getDouble(0.0));

        pidYaxis.setTolerance(epsDistance.getDouble(0.05));
        pidZaxis.setTolerance(epsYaw.getDouble(1.0));
    }

    /** Distância (reta Y) — clamps + deadband para evitar vai-e-volta pequeno */
    public double computeDistancePidOutput() {
        double sp = setpointDistance.getDouble(0.0);
        if (sp == 0.0)
            return 0.0;
        double out = pidYaxis.calculate(getForwardDistance(), sp);
        out = MathUtil.clamp(out, -0.25, 0.25);
        return deadband(out, 0.03);
    }

    /** Yaw (Z) — clamps + deadband */
    public double computeYawPidOutput() {
        double sp = setpointYaw.getDouble(0.0);
        double mult = speedMultiplier.getDouble(1.0);
        double out = pidZaxis.calculate(getYaw(), sp) * mult;
        out = MathUtil.clamp(out, -0.30, 0.30);
        return deadband(out, 0.02);
    }

    /** Slew (rampa) */
    public double applySlewY(double val) {
        return yLimiter.calculate(val);
    }

    public double applySlewZ(double val) {
        return zLimiter.calculate(val);
    }

    /** Reset PIDs e rampas */
    public void resetPids() {
        pidYaxis.reset();
        pidZaxis.reset();
        yLimiter.reset(0.0);
        zLimiter.reset(0.0);
    }

    /** Critérios de término */
    public boolean atYawSetpoint() {
        return pidZaxis.atSetpoint();
    }

    public boolean atDistanceSetpoint() {
        double sp = setpointDistance.getDouble(0.0);
        if (sp == 0.0)
            return true;
        return pidYaxis.atSetpoint();
    }

    // Getters/Setters para comandos
    public double getTargetDistanceMeters() {
        return setpointDistance.getDouble(0.0);
    }

    public double getTargetYawDeg() {
        return setpointYaw.getDouble(0.0);
    }

    public void setYawTargetDeg(double yawDeg) {
        setpointYaw.setDouble(yawDeg);
    }

    public void setDistanceTargetMeters(double metersAbs) {
        setpointDistance.setDouble(metersAbs);
    }

    public void setYawSpeedMultiplier(double mult) {
        speedMultiplier.setDouble(mult);
    }

    @Override
    public void periodic() {
        // Throttle de telemetria (10 Hz)
        double now = Timer.getFPGATimestamp();
        if (now - lastTelemTime > 0.1) {
            leftEncoderValue.setDouble(getLeftEncoderDistance());
            rightEncoderValue.setDouble(getRightEncoderDistance());
            backEncoderValue.setDouble(getBackEncoderDistance());
            gyroValue.setDouble(getYaw());
            forwardDistEntry.setDouble(getForwardDistance());
            lastTelemTime = now;
        }
        // Aplicar alterações de tuning (kP/kI/kD/tolerâncias)
        refreshPidGainsFromDashboard();
    }

    // (stubs que já existiam)
    public void setMotorSpeed(int i) {
    }

    public void setMotorSpeed(double inputRightY) {
    }
}
