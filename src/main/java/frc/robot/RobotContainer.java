package frc.robot;

import frc.robot.gamepad.OI;
import frc.robot.subsystems.DepthCamera;
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

    /** Subsystems */
    private final DepthCamera camera;
    public static DriveBase drivebase;
    public static OI oi;
    public static OMS oms;

    /** Autonomous selection */
    public static SendableChooser<String> autoChooser;
    public static Map<String, AutoCommand> autoMode = new HashMap<>();

    public RobotContainer() {
        // --- Initialize subsystems ---
        camera = new DepthCamera();
        drivebase = new DriveBase();
        oi = new OI();
        oms = new OMS();

        // --- Default commands ---
        drivebase.setDefaultCommand(new Teleop(this));  
        oms.setDefaultCommand(new TeleopOMS());

        // --- Autonomous chooser ---
        autoChooser = new SendableChooser<>();
        autoChooser.setDefaultOption("Drive Forward", "DriveForward");
        autoChooser.addOption("Drive Forward with PID", "DriveFowardWithPid");
        autoChooser.addOption("Rotate180WithPid", "Rotate180WithPid");
        SmartDashboard.putData("Auto Mode", autoChooser);
    }

    /** Getter da DepthCamera para outros comandos */
    public DepthCamera getCamera() {
        return camera;
    }

    /** Retorna o comando autônomo selecionado */
    public Command getAutonomousCommand() {
        String mode = autoChooser.getSelected();
        return autoMode.getOrDefault(mode, new DriveFoward());
    }
}
