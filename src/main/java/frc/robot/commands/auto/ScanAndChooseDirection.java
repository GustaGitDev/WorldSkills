package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.DepthWallRange;

/**
 * Varre esquerda e direita, mede a distância (DepthWallRange) e decide seguir
 * pelo lado com MAIOR espaço livre.
 */

@SuppressWarnings("unused")
public class ScanAndChooseDirection extends SequentialCommandGroup {

    private static final DriveBase drive = RobotContainer.driveBase;
    private static final DepthWallRange wall = RobotContainer.wallRange;

    // parâmetros
    
    private final double stopDistMeters; // p.ex. 0.60
    private final double maxStepMeters; // p.ex. 3.0
    private final double settleSeconds; // p.ex. 0.25
    
    private final double hysteresisMeters; // p.ex. 0.05

    // medições
    private double yaw0;
    private double dLeft = 0.0;
    private double dRight = 0.0;

    public ScanAndChooseDirection(double stopDistMeters, double maxStepMeters, double settleSeconds) {
        this(stopDistMeters, maxStepMeters, settleSeconds, 0.05);
    }

    public ScanAndChooseDirection(double stopDistMeters, double maxStepMeters, double settleSeconds,
            double hysteresisMeters) {
        this.stopDistMeters = stopDistMeters;
        this.maxStepMeters = maxStepMeters;
        this.settleSeconds = settleSeconds;
        this.hysteresisMeters = hysteresisMeters;

        addCommands(
                // snapshot do yaw atual
                new InstantCommand(() -> yaw0 = drive.getYaw()),

                // mede DIREITA (+90)
                new RotateToAngleWithPid(yaw0 + 90.0, null, 6.0), new WaitCommand(settleSeconds),
                new InstantCommand(() -> {
                    wall.update();
                    if (wall.hasValidReading())
                        dRight = wall.getFilteredDistanceMeters();
                    System.out.println("[Scan] RIGHT: " + dRight + " m");
                }),

                // mede ESQUERDA (-90)
                new RotateToAngleWithPid(yaw0 - 90.0, null, 6.0), new WaitCommand(settleSeconds),
                new InstantCommand(() -> {
                    wall.update();
                    if (wall.hasValidReading())
                        dLeft = wall.getFilteredDistanceMeters();
                    System.out.println("[Scan] LEFT:  " + dLeft + " m");
                }),

                // Decide com ConditionalCommand: true = RIGHT, false = LEFT
                new ConditionalCommand(
                        // if-true (RIGHT)
                        new SequentialCommandGroup(new RotateToAngleWithPid(yaw0 + 90.0, null, 6.0),
                                new DriveUntilWallWithPid(stopDistMeters, yaw0 + 90.0, maxStepMeters)),
                        // if-false (LEFT)
                        new SequentialCommandGroup(new RotateToAngleWithPid(yaw0 - 90.0, null, 6.0),
                                new DriveUntilWallWithPid(stopDistMeters, yaw0 - 90.0, maxStepMeters)),
                        // condição
                        () -> chooseRight()));
    }

    /** Retorna true se devemos ir para a direita (com histerese e fallbacks). */
    private boolean chooseRight() {
        // fallbacks quando uma leitura falhou
        if (dRight <= 0 && dLeft > 0)
            return false; // só esquerda válida
        if (dLeft <= 0 && dRight > 0)
            return true; // só direita válida
        // comparação com histerese
        if (dRight > dLeft + hysteresisMeters)
            return true;
        if (dLeft > dRight + hysteresisMeters)
            return false;
        // empate ~ iguais: default = direita
        return true;
    }

    // getters opcionais p/ telemetria
    public double getRightDistanceMeters() {
        return dRight;
    }

    public double getLeftDistanceMeters() {
        return dLeft;
    }
}
