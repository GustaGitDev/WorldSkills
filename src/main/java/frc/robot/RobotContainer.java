/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.gamepad.OI;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.OMS;

import java.util.HashMap;
import java.util.Map;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.Teleop;
import frc.robot.commands.TeleopOMS;
import frc.robot.commands.auto.AutoCommand;
import frc.robot.commands.auto.DriveFoward;

public class RobotContainer {

  /**
   * Create the subsystems and gamepad objects
   */
  public static DriveBase drivebase;
  public static OI oi;
  public static OMS oms;

  public static SendableChooser<String> autoChooser;
  public static Map<String, AutoCommand> autoMode = new HashMap<>();
  

  public RobotContainer()
  {
      //Create new instances
      drivebase = new DriveBase();
      oi = new OI();
      oms = new OMS();
      
      //Set the default command for the training subsytem
      drivebase.setDefaultCommand(new Teleop());  
      oms.setDefaultCommand(new TeleopOMS());
      
       //seleção de instancias
        autoChooser = new SendableChooser<>();
        autoChooser.setDefaultOption("Drive Forward", "DriveForward");
        autoChooser.addOption("Drive Forward with PID", "DriveFowardWithPid");
        autoChooser.addOption("Rotate180WithPid", "Rotate180WithPid");
        SmartDashboard.putData("Auto Mode", autoChooser);  // Isso vai colocar no SmartDashboard
  }

    //@return

    public Command getAutonomousCommand() {
      String mode =autoChooser.getSelected();
      return autoMode.getOrDefault(mode, new DriveFoward());
    }
  }