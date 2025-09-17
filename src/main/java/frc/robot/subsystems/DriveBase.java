package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.studica.frc.TitanQuad;
import com.studica.frc.TitanQuadEncoder;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpiutil.math.MathUtil;

import edu.wpi.first.wpilibj.SlewRateLimiter;
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

    private DigitalInput startBtn;
    private DigitalInput eStopBtn;
    private DigitalOutput runningLED;
    private DigitalOutput stoppedLED;

    public final AHRS navx;

    // === Telemetria (aba existente) ===
    private final ShuffleboardTab tab = Shuffleboard.getTab("Training Robot");
    private final NetworkTableEntry leftEncoderValue = tab.add("left encoder", 0).getEntry();
    private final NetworkTableEntry rightEncoderValue = tab.add("right encoder", 0).getEntry();
    private final NetworkTableEntry backEncoderValue  = tab.add("back encoder", 0).getEntry();
    private final NetworkTableEntry gyroValue         = tab.add("Heading (integrado)", 0).getEntry();
    private final NetworkTableEntry forwardDistEntry  = tab.add("Forward Dist (calc)", 0.0).getEntry();

    // === Tuning PID (aba "Tuning") ===
    private final ShuffleboardTab tuningTab = Shuffleboard.getTab("Tuning");

    private final NetworkTableEntry invertYOutput = tuningTab.add("Invert Y Output", true).getEntry();

    public boolean shouldInvertY() { return invertYOutput.getBoolean(true); }

    private final NetworkTableEntry kP_Y = tuningTab.add("kP_Y", 1.0).withPosition(0, 0).getEntry();
    private final NetworkTableEntry kI_Y = tuningTab.add("kI_Y", 0.0).withPosition(1, 0).getEntry();
    private final NetworkTableEntry kD_Y = tuningTab.add("kD_Y", 0.0).withPosition(2, 0).getEntry();

    private final NetworkTableEntry kP_Z = tuningTab.add("kP_Z", 0.2).withPosition(0, 1).getEntry();
    private final NetworkTableEntry kI_Z = tuningTab.add("kI_Z", 0.0).withPosition(1, 1).getEntry();
    private final NetworkTableEntry kD_Z = tuningTab.add("kD_Z", 0.0).withPosition(2, 1).getEntry();

    private final NetworkTableEntry setpointDistance = tuningTab.add("Setpoint Distance (m)", 0.0).getEntry();
    private final NetworkTableEntry setpointYaw      = tuningTab.add("Setpoint Yaw (deg)", 0.0).getEntry();
    private final NetworkTableEntry epsDistance      = tuningTab.add("Tol Dist (m)", 0.05).getEntry();
    private final NetworkTableEntry epsYaw           = tuningTab.add("Tol Yaw (deg)", 1.0).getEntry();
    private final NetworkTableEntry speedMultiplier  = tuningTab.add("Yaw Speed Mult", 1.5).getEntry();

    // Toggle de inversão do Z
    private final NetworkTableEntry invertZOutput    = tuningTab.add("Invert Z Output", false).getEntry();

    private final PIDController pidYaxis = new PIDController(1, 0, 0);
    public  final PIDController pidZaxis = new PIDController(0.2, 0, 0);

    private final SlewRateLimiter yLimiter = new SlewRateLimiter(1.2);
    public  final SlewRateLimiter zLimiter = new SlewRateLimiter(2.0);

    private boolean autoClosedLoop = false;

    public void setAutoClosedLoop(boolean on) { autoClosedLoop = on; }

    private static double deadband(double v, double db) { return Math.abs(v) < db ? 0.0 : v; }

    private double lastTelemTime = 0.0;

    // === Heading integrado (PITCH_Y como default) ===
    public enum HeadingAxis { ROLL_X, PITCH_Y, YAW_Z }
    private HeadingAxis headingAxis = HeadingAxis.PITCH_Y;

    private double integratedHeadingDeg = 0.0;
    private double lastTs = 0.0;

    private double rateFiltered = 0.0;
    private double gyroBiasDegPerSec = 0.0;

    private static final double RATE_LP_ALPHA = 0.25;
    private static final double STATIONARY_RATE_THRESH = 0.25;
    private static final double BIAS_ADAPT_ALPHA = 0.02;
    private static final double STATIONARY_TIME_TO_ADAPT = 0.4;
    private double stationaryAccum = 0.0;

    public void setHeadingAxis(HeadingAxis axis) { headingAxis = axis; }

    public void zeroIntegratedHeading() {
        integratedHeadingDeg = 0.0;
        gyroBiasDegPerSec = 0.0;
        rateFiltered = 0.0;
        lastTs = Timer.getFPGATimestamp();
    }

    public double getHeadingDeg() { return integratedHeadingDeg; }

    private double getSelectedAxisRateDegPerSec() {
        switch (headingAxis) {
            case ROLL_X:  return navx.getRawGyroX();
            case PITCH_Y: return navx.getRawGyroY();
            case YAW_Z:
            default:      return navx.getRawGyroZ();
        }
    }

    private void updateIntegratedHeading() {
        double now = Timer.getFPGATimestamp();
        if (lastTs == 0.0) { lastTs = now; return; }
        double dt = now - lastTs;
        lastTs = now;
        if (dt <= 0) return;

        double rate = getSelectedAxisRateDegPerSec();
        double rateUnbiased = rate - gyroBiasDegPerSec;
        rateFiltered = rateFiltered + RATE_LP_ALPHA * (rateUnbiased - rateFiltered);

        integratedHeadingDeg += rateFiltered * dt;
        integratedHeadingDeg = Math.IEEEremainder(integratedHeadingDeg, 360.0);

        if (Math.abs(rate) < STATIONARY_RATE_THRESH) {
            stationaryAccum += dt;
            if (stationaryAccum >= STATIONARY_TIME_TO_ADAPT) {
                gyroBiasDegPerSec = (1.0 - BIAS_ADAPT_ALPHA) * gyroBiasDegPerSec
                                  + BIAS_ADAPT_ALPHA * rate;
            }
        } else {
            stationaryAccum = 0.0;
        }
    }

    public DriveBase() {
        leftMotor  = new TitanQuad(Constants.TITAN_ID, Constants.M2);
        rightMotor = new TitanQuad(Constants.TITAN_ID, Constants.M0);
        backMotor  = new TitanQuad(Constants.TITAN_ID, Constants.M1);

        Timer.delay(1.0); // Wait 1s for Titan to configure

        leftEncoder  = new TitanQuadEncoder(leftMotor,  Constants.M2, Constants.WHEEL_DIST_PER_TICK);
        rightEncoder = new TitanQuadEncoder(rightMotor, Constants.M0, Constants.WHEEL_DIST_PER_TICK);
        backEncoder  = new TitanQuadEncoder(backMotor,  Constants.M1, Constants.WHEEL_DIST_PER_TICK);

        startBtn   = new DigitalInput(Constants.START_BUTTON);
        eStopBtn   = new DigitalInput(Constants.E_STOP_SWITCH);
        runningLED = new DigitalOutput(Constants.RUNNING_LED);
        stoppedLED = new DigitalOutput(Constants.STOPPED_LED);

        navx = new AHRS(SPI.Port.kMXP);

        pidYaxis.setTolerance(epsDistance.getDouble(0.05));
        pidZaxis.setTolerance(epsYaw.getDouble(1.0));

        // Continuous input ATIVADO (±180) — recomendado para heading circular
        pidZaxis.enableContinuousInput(-180.0, 180.0);
    }

    // Motores
    public void setLeftmotorspeed(final double speed)  { leftMotor.set(speed); }
    public void setRightmotorspeed(final double speed) { rightMotor.set(speed); }
    public void setBackmotorspeed(final double speed)  { backMotor.set(speed); }
    public void setDriveMotorSpeeds(final double l, final double r, final double b) {
        leftMotor.set(l); rightMotor.set(r); backMotor.set(b);
    }

    public void holonomicDrive(final double x, final double y, final double z) {
        final double yCmd = shouldInvertY() ? -y : y;
        double rightSpeed = ((x/3) - (yCmd/Math.sqrt(3)) + z) * Math.sqrt(3);
        double leftSpeed  = ((x/3) + (yCmd/Math.sqrt(3)) + z) * Math.sqrt(3);
        double backSpeed  = (-2 * x / 3) + z;

        double max = Math.max(Math.abs(rightSpeed), Math.max(Math.abs(leftSpeed), Math.abs(backSpeed)));
        if (max > 1) {
            rightSpeed /= max; leftSpeed /= max; backSpeed /= max;
        }

        if (!autoClosedLoop) {
            double diff = Math.abs(leftSpeed - rightSpeed);
            double adj = 1.0 + diff * 1.0;
            backSpeed *= adj;
            if (Math.abs(leftSpeed) < 0.1)  leftSpeed  *= (1.0 + diff * 0.5);
            if (Math.abs(rightSpeed) < 0.1) rightSpeed *= (1.0 + diff * 0.5);
            leftMotor.set(leftSpeed);
            rightMotor.set(rightSpeed);
            backMotor.set(backSpeed * 1.65);
        } else {
            leftMotor.set(leftSpeed);
            rightMotor.set(rightSpeed);
            backMotor.set(backSpeed);
        }
    }

      /**
     * Turn the Running LED on or off
     * @param on - set to true to turn on the LED
     */
    public void setRunningLED(Boolean on)
    {
        runningLED.set(on);
    }

     /**
     * Turn the stopped LED on or off
     * @param on - set to true to turn on the LED
     */
    public void setStoppedLED(Boolean on)
    {
        stoppedLED.set(on);
    }

    /**
     * Get the value of the start button
     * @return - the current logic of the start button. If wired correctly this will be active low.
     */
    public boolean getStartButton()
    {
        return startBtn.get();
    }

    /**
     * Get the value of the e-stop switch
     * @return - the current logic of the e-stop switch. If wired correctly this will be active low.
     */
    public boolean getEStopButton()
    {
        return eStopBtn.get();
    }

    // Sensores
    public double getLeftEncoderDistance()  { return leftEncoder.getEncoderDistance(); }
    public double getRightEncoderDistance() { return rightEncoder.getEncoderDistance(); }
    public double getBackEncoderDistance()  { return backEncoder.getEncoderDistance(); }

    public double getYaw() { return navx.getRoll(); } // só compatibilidade com código legado

    public void resetEncoders() { leftEncoder.reset(); rightEncoder.reset(); backEncoder.reset(); }
    public void resetYaw() { navx.zeroYaw(); }

    public double getForwardDistance() {
        return (getLeftEncoderDistance() - getRightEncoderDistance()) / 2.0;
    }

    private void refreshPidGainsFromDashboard() {
        pidYaxis.setPID(kP_Y.getDouble(1.0), kI_Y.getDouble(0.0), kD_Y.getDouble(0.0));
        pidZaxis.setPID(kP_Z.getDouble(0.2), kI_Z.getDouble(0.0), kD_Z.getDouble(0.0));
        pidYaxis.setTolerance(epsDistance.getDouble(0.05));
        pidZaxis.setTolerance(epsYaw.getDouble(1.0));
    }

    public double computeDistancePidOutput() {
        double sp = setpointDistance.getDouble(0.0);
        if (sp == 0.0) return 0.0;
        double out = pidYaxis.calculate(getForwardDistance(), sp);
        return deadband(MathUtil.clamp(out, -0.25, 0.25), 0.03);
    }

    public double computeYawPidOutput() {
        double sp   = setpointYaw.getDouble(0.0);
        double mult = speedMultiplier.getDouble(1.5);
        double pv   = getHeadingDeg();

        double out = pidZaxis.calculate(pv, sp) * mult;
        out = MathUtil.clamp(out, -0.50, 0.50);

        // inversão opcional pelo dashboard
        if (invertZOutput.getBoolean(false)) {
            out = -out;
        }
        return deadband(out, 0.01);
    }

    public double applySlewY(double val) { return yLimiter.calculate(val); }
    public double applySlewZ(double val) { return zLimiter.calculate(val); }

    public void resetPids() {
        pidYaxis.reset(); pidZaxis.reset();
        yLimiter.reset(0.0); zLimiter.reset(0.0);
    }

    public boolean atYawSetpoint() { return pidZaxis.atSetpoint(); }
    public boolean atDistanceSetpoint() {
        double sp = setpointDistance.getDouble(0.0);
        return (sp == 0.0) || pidYaxis.atSetpoint();
    }

    public double getTargetDistanceMeters() { return setpointDistance.getDouble(0.0); }
    public double getTargetYawDeg()         { return setpointYaw.getDouble(0.0); }

    public void setYawTargetDeg(double yawDeg) {
        yawDeg = Math.IEEEremainder(yawDeg, 360.0);
        setpointYaw.setDouble(yawDeg);
    }

    public void setDistanceTargetMeters(double m) { setpointDistance.setDouble(m); }
    public void setYawSpeedMultiplier(double mult) { speedMultiplier.setDouble(mult); }

    @Override
    public void periodic() {
        // Atualiza heading integrado
        updateIntegratedHeading();

        // Throttle de telemetria (10 Hz p/ encoders e heading básico)
        double now = Timer.getFPGATimestamp();
        if (now - lastTelemTime > 0.1) {
            leftEncoderValue.setDouble(getLeftEncoderDistance());
            rightEncoderValue.setDouble(getRightEncoderDistance());
            backEncoderValue.setDouble(getBackEncoderDistance());
            gyroValue.setDouble(getHeadingDeg()); // heading integrado usado pelo PID
            forwardDistEntry.setDouble(getForwardDistance());
            lastTelemTime = now;
        }

        // Atualiza ganhos PID do Shuffleboard
        refreshPidGainsFromDashboard();

        // === Debug extra: sensores brutos e PID ===
        SmartDashboard.putNumber("gyroX (deg/s)", navx.getRawGyroX());
        SmartDashboard.putNumber("gyroY (deg/s)", navx.getRawGyroY());
        SmartDashboard.putNumber("gyroZ (deg/s)", navx.getRawGyroZ());

        SmartDashboard.putNumber("Heading pv", getHeadingDeg());           // process variable
        SmartDashboard.putNumber("Heading sp", getTargetYawDeg());         // setpoint
        SmartDashboard.putNumber("Heading err", pidZaxis.getPositionError());
    }

    public void setMotorSpeed(int i) {}
    public void setMotorSpeed(double inputRightY) {}
}
