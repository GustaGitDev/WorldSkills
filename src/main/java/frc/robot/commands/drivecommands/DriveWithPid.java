package frc.robot.commands.drivecommands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveBase;

public class DriveWithPid extends CommandBase {
    private static final DriveBase drive = RobotContainer.drivebase;

    private double setpointDistance;  // Distância desejada
    private double setpointYaw;  // Ângulo desejado (Yaw)
    private double speedMultiplier;  // Fator de multiplicação para a velocidade de rotação

    PIDController pidYaxis;  // PID para controle da distância
    PIDController pidZaxis;  // PID para controle da rotação (Yaw)

    public DriveWithPid(double setpointDistance, double epsilonDistance, double setpointYaw, double epsilonYaw, double speedMultiplier) {
        this.setpointDistance = setpointDistance;
        this.setpointYaw = setpointYaw;
        this.speedMultiplier = speedMultiplier;
        addRequirements(drive);

        // PID para movimento linear (não usado no caso de rotação)
        pidYaxis = new PIDController(1, 0, 0);  
        pidYaxis.setTolerance(epsilonDistance);

        // PID para rotação (Yaw)
        pidZaxis = new PIDController(0.1, 0, 0);  
        pidZaxis.setTolerance(epsilonYaw);
    }

    @Override
    public void initialize() {
        drive.resetEncoders();  // Reseta os encoders no início do movimento
        drive.resetYaw();  // Reseta o yaw para garantir que começamos de 0 graus
        pidYaxis.reset();
        pidZaxis.reset();
    }

    @Override
    public void execute() {
        // Se estamos apenas controlando a rotação, desative o PID de distância
        double pidOutputY = 0.0;

        // Calcula o PID para rotação (Yaw) e ajusta a velocidade com o multiplicador
        double pidOutputZ = MathUtil.clamp(pidZaxis.calculate(drive.getYaw(), setpointYaw) * speedMultiplier, -1.0, 1.0);

        // Se o setpointDistance for diferente de 0, use o PID para distância
        if (setpointDistance != 0.0) {
            pidOutputY = MathUtil.clamp(pidYaxis.calculate(drive.getAverageForwardEncoderDistance(), setpointDistance), -0.5, 0.5);
        }

        // Controla o movimento linear e a rotação simultaneamente
        drive.holonomicDrive(0.0, pidOutputY, pidOutputZ);  // Controle linear (Y) e rotação (Z)

        // Se a rotação ou o movimento linear atingir o setpoint, pare o robô
        if (Math.abs(drive.getYaw() - setpointYaw) <= 1.0 && Math.abs(drive.getAverageForwardEncoderDistance() - setpointDistance) <= 0.1) {
            drive.setDriveMotorSpeeds(0.0, 0.0, 0.0);  // Para o robô quando atingir tanto a rotação quanto a distância
        }
    }

    @Override
    public void end(boolean interrupted) {
        drive.setDriveMotorSpeeds(0.0, 0.0, 0.0);  // Se interromper, pare o robô
    }

    @Override
    public boolean isFinished() {
        // O comando termina quando o yaw atingir o setpoint com a tolerância definida
        // A verificação se o yaw está dentro da tolerância é suficiente para encerrar o comando
        return pidZaxis.atSetpoint();  // Foca apenas na rotação e não na distância
    }
}
