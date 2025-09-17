package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import com.studica.frc.MockDS;

public class Robot extends TimedRobot {

  private MockDS ds;
  private boolean active = false;
  private int countLED;
  private boolean prevLEDValue;

  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() 
  {
    m_robotContainer = new RobotContainer();
    ds = new MockDS();
    RobotContainer.drivebase.setRunningLED(false);
    RobotContainer.drivebase.setStoppedLED(false);
    countLED = 1;
    prevLEDValue = true;
   
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    if (RobotContainer.driveBase.getStartButton() && !active) {
      ds.enable();
      active = true;
  }
    if (RobotContainer.driveBase.getEStopButton() && active) {
      ds.disable();
      active = false;
    }
  }

  @Override
  public void disabledInit() {
    RobotContainer.driveBase.setRunningLED(true);
    RobotContainer.driveBase.setStoppedLED(false);
  }

  @Override
  public void disabledPeriodic() { }

  @Override
  public void autonomousInit() {
    RobotContainer.driveBase.setRunningLED(false);
    Command autonomousCommand = m_robotContainer.getAutonomousCommand();
    if (autonomousCommand != null) {
      autonomousCommand.schedule(); // correção 27/08 acionamento auto
    }
  }

  @Override
  public void autonomousPeriodic() { 
    runningLED();
  }

  @Override
  public void teleopInit() {
    Command autonomousCommand = m_robotContainer.getAutonomousCommand();
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
      RobotContainer.driveBase.setRunningLED(false);
    }

  @Override
  public void teleopPeriodic() {
    runningLED();
   }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() { 
  }
  @Override
  public void simulationInit() { }

  @Override
  public void simulationPeriodic() { }
  
  public void runningLED(){
      if ((countLED % 25) == 0) {
          if (prevLEDValue) {
              RobotContainer.driveBase.setRunningLED(false);
              prevLEDValue = false; 
            } else {
              RobotContainer.driveBase.setRunningLED(true);
              prevLEDValue = true;
          }
          countLED = 0;   
         } else {
          countLED++;}
    }
}
