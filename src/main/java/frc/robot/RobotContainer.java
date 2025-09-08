package frc.robot;

import frc.robot.gamepad.OI;
import frc.robot.subsystems.DepthCamera;
import frc.robot.subsystems.DepthWallRange;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.Monitoramento;
import frc.robot.subsystems.OMS;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.commands.Teleop;
import frc.robot.commands.TeleopOMS;

import frc.robot.commands.auto.DriveFoward; // open-loop existente
import frc.robot.commands.auto.DriveFowardWithPid; // legado (opcional no menu)
import frc.robot.commands.auto.DriveStraightWithPid; // wrapper novo (reta PID)
import frc.robot.commands.auto.LabirintoA;
import frc.robot.commands.auto.RotateToAngleWithPid; // wrapper novo (giro PID)
import frc.robot.commands.auto.AutonomoTESTE; // triângulo

public class RobotContainer {

    /** === Subsystems estáticos de uso global === */
    public static final DriveBase driveBase = new DriveBase();
    public final Monitoramento monitor = new Monitoramento();
    // Alias para compatibilidade com código que usa "drivebase"
    public static final DriveBase drivebase = driveBase;

    public static final DepthCamera camera = new DepthCamera();
    public static final DepthWallRange wallRange = new DepthWallRange(camera);

    /** Outros subsistemas/controles (se quiser manter estáticos) */
    public static OI oi;
    public static OMS oms;

    /** Autonomous selection (centralizado aqui) */
    public static final SendableChooser<String> autoChooser = new SendableChooser<>();
    public static final Map<String, Command> autoMode = new HashMap<>();

    public RobotContainer() {
        // Parâmetros iniciais do filtro da parede (ajuste se quiser)
        wallRange.setParams(10, 2, 0.2); // ROI 21x21, passo 2, EMA alpha=0.2

        // Inicializa OI/OMS
        oi = new OI();
        oms = new OMS();

        // Default commands
        driveBase.setDefaultCommand(new Teleop(this));
        oms.setDefaultCommand(new TeleopOMS());

        // ===== AUTONOMOUS MENU (value == label) =====
        autoChooser.setDefaultOption("Drive Forward (open-loop)", "Drive Forward (open-loop)");
        autoMode.put("Drive Forward (open-loop)", new DriveFoward());

        autoChooser.addOption("Drive Straight PID 1.0m @0°", "Drive Straight PID 1.0m @0°");
        autoMode.put("Drive Straight PID 1.0m @0°", new DriveStraightWithPid(1.0, 0.0, null, 6));

        autoChooser.addOption("Drive Straight PID 2.0m @0°", "Drive Straight PID 2.0m @0°");
        autoMode.put("Drive Straight PID 2.0m @0°", new DriveStraightWithPid(2.0, 0.0, null, 20));

        autoChooser.addOption("Rotate to 90°", "Rotate to 90°");
        autoMode.put("Rotate to 90°", new RotateToAngleWithPid(90.0, null, 5));

        autoChooser.addOption("Rotate to 180°", "Rotate to 180°");
        autoMode.put("Rotate to 180°", new RotateToAngleWithPid(180.0, null, 7));

        autoChooser.addOption("Triangle Test", "Triangle Test");
        autoMode.put("Triangle Test", new AutonomoTESTE());

        autoChooser.addOption("Modulo A", "Modulo A");
        autoMode.put("Modulo A", new LabirintoA());

        // (Opcional) manter wrapper legado visível
        autoChooser.addOption("DriveFowardWithPid (legacy)", "DriveFowardWithPid (legacy)");
        autoMode.put("DriveFowardWithPid (legacy)", new DriveFowardWithPid());

        // Publica no dashboard
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
