package frc.vision.nt;

import edu.wpi.first.networktables.*;
import frc.vision.config.VisionConfig;

public class VisionPublisher implements AutoCloseable {
  private final NetworkTableInstance inst;
  private final NetworkTable table;
  private final NetworkTableEntry hasTarget, tx, ty, conf;

  public VisionPublisher() {
    inst = NetworkTableInstance.getDefault();

    // NT3 (WPILib 2020): cliente na RPi → servidor no roboRIO (10.12.34.2)
    inst.startClient(VisionConfig.NT_IDENTITY);
    inst.setServer(VisionConfig.NT_SERVER); // usa o IP do seu roboRIO

    table = inst.getTable(VisionConfig.NT_TABLE);
    hasTarget = table.getEntry("hasTarget");
    tx        = table.getEntry("x");
    ty        = table.getEntry("y");
    conf      = table.getEntry("conf");
  }

  public void publishNoTarget() {
    hasTarget.setBoolean(false);
  }

  public void publish(double nx, double ny, double confidence) {
    hasTarget.setBoolean(true);
    tx.setDouble(nx);   // 0..1
    ty.setDouble(ny);
    conf.setDouble(confidence);
  }

  @Override public void close() { inst.stopClient(); }
}
