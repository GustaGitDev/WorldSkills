package frc.robot.subsystems;

import com.studica.frc.Servo;
import com.studica.frc.TitanQuad;
import com.studica.frc.TitanQuadEncoder;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class OMS extends SubsystemBase
 {
    private TitanQuad elevator;
    public Servo claw;

    private TitanQuadEncoder elevatorEncoder;

    private ShuffleboardTab tab = Shuffleboard.getTab("Training Robot");
    private NetworkTableEntry elevatorEncoderValue = tab.add("Elevator Encoder", 0).getEntry();

    public OMS()
    {
        elevator = new TitanQuad(Constants.TITAN_ID, Constants.M3);
        claw = new Servo(Constants.DIF_SERVO);

        elevatorEncoder = new TitanQuadEncoder(elevator, Constants.DIF_SERVO, Constants.ELEVATOR_DIST_TICK);
    }
        

    public void setElevatorMotorSpeed(double speed)
{
    elevator.set(speed);
}

public double getElevatorEncoderDistance(){
    return elevatorEncoder.getEncoderDistance();
    }
    public void setServoPosition( double degrees)
    {
        claw.setAngle(degrees);
    }
    public void resetEncoders()
    {
        elevatorEncoder.reset();
    }
    @Override
    public void periodic(){
        elevatorEncoderValue.setDouble(getElevatorEncoderDistance());
    }
}
