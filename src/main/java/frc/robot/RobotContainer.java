package frc.robot;

import frc.robot.gamepad.OI;
import frc.robot.subsystems.DepthCamera;
import frc.robot.subsystems.DepthWallRange;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.Monitoramento;
import frc.robot.subsystems.OMS;
import frc.robot.subsystems.VisionSubsystem;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
// +++ novo:
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.commands.Teleop;
import frc.robot.commands.TeleopOMS;
import frc.robot.commands.auto.DriveStraightWithPid; 
import frc.robot.commands.auto.LabirintoA;
import frc.robot.commands.auto.RotateToAngleWithPid;
import frc.robot.commands.auto.AutonomoTESTE;

public class RobotContainer {

    /** === Subsystems estáticos de uso global === */
    public static final DriveBase driveBase = new DriveBase();
    public final Monitoramento monitor = new Monitoramento();
    // Alias para compatibilidade com código que usa "drivebase"
    public static final DriveBase drivebase = driveBase;
    public static final DepthCamera camera = new DepthCamera();
    public static final VisionSubsystem vision = new VisionSubsystem(camera);
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

        // Se você adotou ROLL como “eixo de giro” físico, mantenha ROLL_X:
        driveBase.setHeadingAxis(DriveBase.HeadingAxis.PITCH_Y);
        // Se voltar a usar o yaw “normal” do robô, troque para:
        // driveBase.setHeadingAxis(DriveBase.HeadingAxis.YAW_Z);

        // Botão util no Dashboard para zerar o heading integrado quando quiser:
        SmartDashboard.putData("Zero Integrated Heading",
            new InstantCommand(() -> driveBase.zeroIntegratedHeading()));
        // ================================================================

        // Inicializa OI/OMS
        oi = new OI();
        oms = new OMS();

        // Default commands
        driveBase.setDefaultCommand(new Teleop(this));
        oms.setDefaultCommand(new TeleopOMS());

        // ===== AUTONOMOUS MENU (value == label) =====
        autoChooser.addOption("Reto PID", "Frente Pid");
        autoMode.put("Frente Pid", new DriveStraightWithPid(1.0, 0.0, null, 6));

        autoChooser.addOption("Giro to 90°", "Giro to 90°");
        autoMode.put("Giro to 90°", new RotateToAngleWithPid(90.0, null, 5));

        autoChooser.addOption("Auto Test", "Auto Test");
        autoMode.put("Auto Test", new AutonomoTESTE());

        autoChooser.addOption("Modulo A", "Modulo A");
        autoMode.put("Modulo A", new LabirintoA());

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
        return autoMode.getOrDefault(mode, new LabirintoA());
    }
}
