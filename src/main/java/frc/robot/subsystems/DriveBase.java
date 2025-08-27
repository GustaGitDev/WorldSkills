package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveBase extends SubsystemBase {
  // Ajuste as portas PWM conforme ligou no VMX ou RoboRIO
  private final PWMVictorSPX motorA = new PWMVictorSPX(0); // roda em 0°
  private final PWMVictorSPX motorB = new PWMVictorSPX(1); // roda em 120°
  private final PWMVictorSPX motorC = new PWMVictorSPX(2); // roda em 240°

  public DriveBase() {
    motorA.setInverted(false);
    motorB.setInverted(false);
    motorC.setInverted(false);
  }

  /** Kiwi Drive (3 rodas omni a 120°) */
  public void kiwiDrive(double x, double y, double rot) {
    double mA = x + rot;
    double mB = -0.5 * x + (Math.sqrt(3) / 2.0) * y + rot;
    double mC = -0.5 * x - (Math.sqrt(3) / 2.0) * y + rot;

    // normaliza
    double max = Math.max(1.0, Math.max(Math.abs(mA), Math.max(Math.abs(mB), Math.abs(mC))));
    mA /= max;
    mB /= max;
    mC /= max;

    motorA.set(mA);
    motorB.set(mB);
    motorC.set(mC);
  }

  public void stop() {
    motorA.set(0.0);
    motorB.set(0.0);
    motorC.set(0.0);
  }

  @Override
  public void periodic() {
    // opcional
  }

  // Substitui o antigo holonomicDrive
public void holonomicDrive(double x, double y, double z) {
    kiwiDrive(x, y, z);
  }
  
  // Substitui o antigo setDriveMotorSpeeds
  public void setDriveMotorSpeeds(double a, double b, double c) {
    motorA.set(a);
    motorB.set(b);
    motorC.set(c);
  }
  
  // Substitui o antigo resetEncoders (sem encoder real ainda)
  public void resetEncoders() {
    // TODO: implementar se adicionar encoders
  }
  
  // Substitui o antigo resetYaw (sem giroscópio ainda)
  public void resetYaw() {
    // TODO: implementar se adicionar IMU
  }
  
  // Substitui o antigo getAverageForwardEncoderDistance
  public double getAverageForwardEncoderDistance() {
    return 0.0; // placeholder
  }
  
  // Substitui o antigo getYaw
  public double getYaw() {
    return 0.0; // placeholder
  }  

}
