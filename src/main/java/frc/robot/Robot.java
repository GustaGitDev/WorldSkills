/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.auto.AutoCommand;
import frc.robot.commands.auto.DriveFoward;
import frc.robot.commands.auto.DriveFowardWithPid;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private RobotContainer m_robotContainer;
  private Command autonomousCommand;
  private final Timer autoTimer = new Timer();
  private boolean autoActive = false;


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer. 
    m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Garante que, se você usar Commands em qualquer lugar, eles rodem.
    CommandScheduler.getInstance().run();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {
    if (null == RobotContainer.autoChooser)
    {
        RobotContainer.autoChooser = new SendableChooser<>();
    }
    RobotContainer.autoChooser.setDefaultOption("Drive Foward", "Drive Forward");
    RobotContainer.autoMode.put("Drive Foward", new DriveFoward());
    AddAutoMode(RobotContainer.autoChooser, "Drive Foward withPID", new DriveFowardWithPid());
    SmartDashboard.putData(RobotContainer.autoChooser);
  }

  public void AddAutoMode(SendableChooser<String>chooser, String auto, AutoCommand cmd){
    chooser.addOption(auto, auto);
    RobotContainer.autoMode.put(auto, cmd);
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    // (Opcional) Se você agendava um comando de auto, pode comentar por enquanto:
    // Command auto = m_robotContainer.getAutonomousCommand();
    // if (auto != null) auto.schedule();
  
    // Fallback por tempo: liga o timer e ativa a flag
    autoTimer.reset();
    autoTimer.start();
    autoActive = true;
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    if (!autoActive) return;
  
    double t = autoTimer.get();
    if (t < 2.0) {
      // Anda pra frente enquanto estiver nos 2s iniciais.
      // holonomicDrive(x=strafe, y=frente, z=giro).
      // Se seu drive estiver invertido, troque 0.5 por -0.5.
      frc.robot.RobotContainer.drivebase.kiwiDrive(0.0, -0.5, 0.0);
      // Alternativa caso não mexa: experimente -0.5
      // frc.robot.RobotContainer.drivebase.holonomicDrive(0.0, -0.5, 0.0);
    } else {
      // Para motores e encerra o auto
      frc.robot.RobotContainer.drivebase.setDriveMotorSpeeds(0.0, 0.0, 0.0);
      autoActive = false;
    }
  }

  @Override
  public void teleopInit() {
    // Garante que nada do auto continue rodando
    autoActive = false;
    frc.robot.RobotContainer.drivebase.setDriveMotorSpeeds(0.0, 0.0, 0.0);
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

  /**
   * added here to satisfy the watchdog
   */
  @Override
  public void simulationInit(){
  }
  
  /**
   * added here to satisfy the watchdog
   */ 
  @Override
  public void simulationPeriodic(){
  }
}
