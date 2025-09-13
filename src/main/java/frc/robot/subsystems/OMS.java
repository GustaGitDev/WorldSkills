package frc.robot.subsystems;

import com.studica.frc.Servo;
import com.studica.frc.TitanQuad;
import com.studica.frc.TitanQuadEncoder;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class OMS extends SubsystemBase {
    private TitanQuad elevator;
    private TitanQuadEncoder elevatorEncoder;

    // Servos
    private final Servo claw;
    private final Servo linearGarra;
    private final Servo garraRot;

    // Se seus servos forem de 180°, deixe 180. Se forem de 270°, mude para 270/300 conforme datasheet.
    private static final double SERVO_MIN_DEG = 0.0;
    private static final double SERVO_MAX_DEG = 180.0;

    // (Opcional) posições padrão
    public static final double CLAW_OPEN_DEG = 120.0;
    public static final double CLAW_CLOSE_DEG = 60.0;

    private ShuffleboardTab tab = Shuffleboard.getTab("Training Robot");
    private NetworkTableEntry elevatorEncoderValue = tab.add("Elevator Encoder", 0).getEntry();

    public OMS() {
        elevator = new TitanQuad(Constants.TITAN_ID, Constants.M3);
        claw = new Servo(Constants.GARSERVO);
        linearGarra = new Servo(Constants.LINEARGARRA);
        garraRot = new Servo(Constants.ROTGARRA);

        // ⚠️ Verifique: aqui você estava usando Constants.GARSERVO (canal do SERVO) no encoder!
        // Corrija para a porta/canal correta do encoder (ex.: Constants.ELEVATOR_ENCODER_CH):
        elevatorEncoder = new TitanQuadEncoder(elevator, Constants.M3, Constants.ELEVATOR_DIST_TICK);
    }

    /* ===== Elevator ===== */
    public void setElevatorMotorSpeed(double speed) { elevator.set(speed); }
    public double getElevatorEncoderDistance(){ return elevatorEncoder.getEncoderDistance(); }
    public void resetEncoders(){ elevatorEncoder.reset(); }

    /* ===== Util ===== */
    private double clampDeg(double deg) {
        if (deg < SERVO_MIN_DEG) return SERVO_MIN_DEG;
        if (deg > SERVO_MAX_DEG) return SERVO_MAX_DEG;
        return deg;
    }

    /* ===== Setters individuais ===== */
    public void setClawDeg(double deg) {
        claw.setAngle(clampDeg(deg));
    }

    public void setLinearGarraDeg(double deg) {
        linearGarra.setAngle(clampDeg(deg));
    }

    public void setGarraRotDeg(double deg) {
        garraRot.setAngle(clampDeg(deg));
    }

    /* ===== Setter combinado (parâmetros independentes) ===== */
    public void setServos(Double clawDeg, Double linearDeg, Double rotDeg) {
        if (clawDeg != null) setClawDeg(clawDeg);
        if (linearDeg != null) setLinearGarraDeg(linearDeg);
        if (rotDeg != null) setGarraRotDeg(rotDeg);
    }

    @Override
    public void periodic(){
        elevatorEncoderValue.setDouble(getElevatorEncoderDistance());
    }
}
