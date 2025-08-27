package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.studica.frc.TitanQuad;
import com.studica.frc.TitanQuadEncoder;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveBase extends SubsystemBase {
    
    private final TitanQuad leftMotor;
    private final TitanQuad rightMotor;
    private final TitanQuad backMotor;

    private final TitanQuadEncoder leftEncoder;
    private final TitanQuadEncoder rightEncoder;
    private final TitanQuadEncoder backEncoder;

    private final AHRS navx;

    private final ShuffleboardTab tab = Shuffleboard.getTab("Training Robot");
    private final NetworkTableEntry leftEncoderValue = tab.add("left enconder", 0).getEntry();
    private final NetworkTableEntry rightEncoderValue = tab.add("right enconder", 0).getEntry();
    private final NetworkTableEntry backEncoderValue = tab.add("back enconder", 0).getEntry();
    private final NetworkTableEntry gyroValue = tab.add("NavX Yaw", 0).getEntry();

    public DriveBase() {

        leftMotor = new TitanQuad(Constants.TITAN_ID, Constants.M0);
        rightMotor = new TitanQuad(Constants.TITAN_ID, Constants.M1);
        backMotor = new TitanQuad(Constants.TITAN_ID, Constants.M2);

        leftEncoder = new TitanQuadEncoder(leftMotor, Constants.M0, Constants.WHEEL_DIST_PER_TICK);
        rightEncoder = new TitanQuadEncoder(rightMotor, Constants.M1, Constants.WHEEL_DIST_PER_TICK);
        backEncoder = new TitanQuadEncoder(backMotor, Constants.M2, Constants.WHEEL_DIST_PER_TICK);

        leftEncoder.setReverseDirection();
        

        navx = new AHRS(SPI.Port.kMXP);

    }

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
        double rightSpeed = ((x / 3) - (y / Math.sqrt(3)) + z) * Math.sqrt(3);
        double leftSpeed = ((x / 3) + (y / Math.sqrt(3)) + z) * Math.sqrt(3);
        double backSpeed = (-2 * x / 3) + z;
    
        double max = Math.abs(rightSpeed);
        if (Math.abs(leftSpeed) > max) max = Math.abs(leftSpeed);
        if (Math.abs(backSpeed) > max) max = Math.abs(backSpeed);
    
        if (max > 1) {
            rightSpeed /= max;
            leftSpeed /= max;
            backSpeed /= max;
        }
    
        // Ajuste de calibração para alinhar os motores
        double motorAdjustmentFactor = 1.0; // Ajuste o valor para compensar qualquer diferença
        if (Math.abs(leftSpeed) < 0.1) leftSpeed *= motorAdjustmentFactor; 
        if (Math.abs(rightSpeed) < 0.1) rightSpeed *= motorAdjustmentFactor;
    
        leftMotor.set(leftSpeed);
        rightMotor.set(rightSpeed);
        backMotor.set(backSpeed);
    }

    public double getleftEncoderDistance()
    {
        return leftEncoder.getEncoderDistance();
    }

    public double getRightEncoderDistance()
    {
        return rightEncoder.getEncoderDistance();
    }

    public double getBackEncoderDistance()
    {
        return backEncoder.getEncoderDistance();
    }

    public double getAverageForwardEncoderDistance(){
        return (getleftEncoderDistance()+getRightEncoderDistance())/2;
        
    }

    public double getYaw()
    {
        return navx.getYaw();
    }

    public void resetEncoders()
    {
        leftEncoder.reset();
        rightEncoder.reset();
        backEncoder.reset();
    }

    public void resetYaw()
        {
            navx.zeroYaw();
        }
    
        @Override
        public void periodic()
        {
            leftEncoderValue.setDouble(getleftEncoderDistance());
            rightEncoderValue.setDouble(getRightEncoderDistance());
            backEncoderValue.setDouble(getBackEncoderDistance());
            gyroValue.setDouble(getYaw());

        }
	public void setMotorSpeed(int i) {
	}

	public void setMotorSpeed(double inputRightY) {
	}


    



}
