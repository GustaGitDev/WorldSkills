package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.*;

public class VisionSubsystem extends SubsystemBase {
  private final NetworkTableEntry hasTarget, x, y, conf;

  private volatile boolean has;
  private volatile double nx, ny, confidence;

  public VisionSubsystem() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("vision");
    hasTarget = table.getEntry("hasTarget");
    x         = table.getEntry("x");
    y         = table.getEntry("y");
    conf      = table.getEntry("conf");
  }

  @Override
  public void periodic() {
    has       = hasTarget.getBoolean(false);
    nx        = x.getDouble(0.5);
    ny        = y.getDouble(0.5);
    confidence= conf.getDouble(0.0);
  }

  public boolean hasTarget()  { return has; }
  public double getNx()       { return nx; } // 0..1
  public double getNy()       { return ny; }
  public double getConf()     { return confidence; }
}