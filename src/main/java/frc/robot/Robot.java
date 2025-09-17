/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import com.studica.frc.MockDS;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private MockDS ds;
  private boolean active = false;
  private int countLED;
  private boolean prevLEDValue;

  private RobotContainer m_robotContainer;
  private long lastFrameNs = System.nanoTime(); 

  /** This function is run when the robot is first started up and should be used for any init code. */
  @Override
  public void robotInit() 
  {
    // Instantiate our RobotContainer.
    m_robotContainer = new RobotContainer();
    ds = new MockDS(); // Create the instance
    RobotContainer.drivebase.setRunningLED(false);
    RobotContainer.drivebase.setStoppedLED(false);
    countLED = 1;
    prevLEDValue = true;
   
  }

  /** This function is called every robot packet, no matter the mode. */
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    if (!RobotContainer.driveBase.getStartButton() && !active)
    {
      ds.enable();
      active = true;
    }
    // If E-Stop button is pushed disable the robot
    if (!RobotContainer.driveBase.getEStopButton() && active)
    {
      ds.disable();
      active = false;
    }
  }


  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    RobotContainer.driveBase.setRunningLED(true);
    RobotContainer.driveBase.setStoppedLED(false);
    // NÃO registrar/alterar o chooser aqui. Tudo é feito no RobotContainer.
  }

  @Override
  public void disabledPeriodic() { }

  /** This autonomous runs the autonomous command selected by your RobotContainer class. */
  @Override
  public void autonomousInit() {
    RobotContainer.driveBase.setRunningLED(false);
    Command autonomousCommand = m_robotContainer.getAutonomousCommand();
    if (autonomousCommand != null) {
      autonomousCommand.schedule(); // correção 27/08 acionamento auto
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() { 
    runningLED();
  }

  @Override
  public void teleopInit() {
     // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    Command autonomousCommand = m_robotContainer.getAutonomousCommand();
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
      RobotContainer.driveBase.setRunningLED(false);
    }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    runningLED();
   }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() { }

  /** added here to satisfy the watchdog */
  @Override
  public void simulationInit() { }

  /** added here to satisfy the watchdog */
  @Override
  public void simulationPeriodic() { }
  
  public void runningLED()
  {
      if ((countLED % 25) == 0)
      {
          if (prevLEDValue)
          {
              RobotContainer.driveBase.setRunningLED(false);
              prevLEDValue = false;
          }
          else
          {
              RobotContainer.driveBase.setRunningLED(true);
              prevLEDValue = true;
          }
          countLED = 1;   
      }
      else
      {
          countLED++;
      }
    }

}
