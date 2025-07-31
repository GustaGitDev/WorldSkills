package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.gamepad.OI;
import frc.robot.subsystems.DriveBase;

public class Teleop extends CommandBase {

    private static final DriveBase drivebase = RobotContainer.drivebase;
    private static final OI oi = RobotContainer.oi;

    double inputLeftY = 0;
    double inputLeftX = 0;
    double inputRightY = 0;
    double inputRightX = 0;

    // Rampagem
    double prevLeftY = 0;
    double prevLeftX = 0;
    double prevRightY = 0;
    double prevRightX = 0;

    private static final double RAMP_UP = 0.05;
    private static final double RAMP_DOWN = 0.05;
    private static final double DELTA_LIMIT = 0.075;
    private static final double DEADZONE = 0.1;

    public Teleop() {
        addRequirements(drivebase);
    }

    @Override
    public void initialize() {
        drivebase.resetEncoders();
    }

    @Override
    public void execute() {
        // Leitura do controle
        inputLeftX = applyDeadzone(oi.getLeftDriveX());
        inputLeftY = applyDeadzone(oi.getLeftDriveY());
        inputRightX = applyDeadzone(oi.getRightDriveX());
        inputRightY = applyDeadzone(oi.getRightDriveY());

        // Rampagem simples
        inputLeftX = ramp(inputLeftX, prevLeftX);
        inputLeftY = ramp(inputLeftY, prevLeftY);
        inputRightX = ramp(inputRightX, prevRightX);
        inputRightY = ramp(inputRightY, prevRightY);

        // Atualiza os valores anteriores
        prevLeftX = inputLeftX;
        prevLeftY = inputLeftY;
        prevRightX = inputRightX;
        prevRightY = inputRightY;

        // Comando de direção
        drivebase.holonomicDrive(inputRightY, inputLeftX, inputRightX);
        if (inputLeftX == 0 && inputLeftY == 0 && inputRightX == 0 && inputRightY == 0) {
            drivebase.setDriveMotorSpeeds(0.0, 0.0, 0.0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        drivebase.setDriveMotorSpeeds(0.0, 0.0, 0.0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    // Aplica zona morta
    private double applyDeadzone(double value) {
        return Math.abs(value) < DEADZONE ? 0.0 : value;
    }

    // Função de rampagem
    private double ramp(double current, double previous) {
        double delta = current - previous;
    
        // Se o valor de destino é zero, zera mais rápido
        if (Math.abs(current) < DEADZONE) return 0.0;
    
        if (Math.abs(delta) < DELTA_LIMIT) return current;
        if (delta > 0) return previous + RAMP_UP;
        else return previous - RAMP_DOWN;
    }
}
