package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.gamepad.OI;
import frc.robot.subsystems.DriveBase;
import frc.robot.RobotContainer;

public class Teleop extends CommandBase {

    private final DriveBase drivebase;
    private final OI oi;
    

    private double inputLeftY = 0;
    private double inputLeftX = 0;
    private double inputRightY = 0;
    private double inputRightX = 0;

    private double prevLeftY = 0;
    private double prevLeftX = 0;
    private double prevRightY = 0;
    private double prevRightX = 0;

    private static final double RAMP_UP = 0.05;
    private static final double RAMP_DOWN = 0.05;
    private static final double DELTA_LIMIT = 0.075;
    private static final double DEADZONE = 0.1;

    // Construtor recebendo RobotContainer
    public Teleop(RobotContainer container) {
        
        this.drivebase = RobotContainer.drivebase;
        this.oi = RobotContainer.oi;
        addRequirements(drivebase);
    }

    @Override
    public void initialize() {
        drivebase.resetEncoders();
    }

    @Override
    public void execute() {
        // Leitura dos joysticks
        inputLeftX = applyDeadzone(oi.getLeftDriveX());
        inputLeftY = applyDeadzone(oi.getLeftDriveY());
        inputRightX = applyDeadzone(oi.getRightDriveX());
        inputRightY = applyDeadzone(oi.getRightDriveY());

        // Rampagem
        inputLeftX = ramp(inputLeftX, prevLeftX);
        inputLeftY = ramp(inputLeftY, prevLeftY);
        inputRightX = ramp(inputRightX, prevRightX);
        inputRightY = ramp(inputRightY, prevRightY);

        prevLeftX = inputLeftX;
        prevLeftY = inputLeftY;
        prevRightX = inputRightX;
        prevRightY = inputRightY;

        // Movimento do robô
        drivebase.holonomicDrive(inputRightY, inputLeftX, inputRightX);

        // Para o robô caso os joysticks estejam neutros
        if (inputLeftX == 0 && inputLeftY == 0 && inputRightX == 0 && inputRightY == 0) {
            drivebase.setDriveMotorSpeeds(0, 0, 0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        drivebase.setDriveMotorSpeeds(0, 0, 0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    // --- Zona morta ---
    private double applyDeadzone(double value) {
        return Math.abs(value) < DEADZONE ? 0 : value;
    }

    // --- Rampagem ---
    private double ramp(double current, double previous) {
        double delta = current - previous;
        if (Math.abs(current) < DEADZONE) return 0;
        if (Math.abs(delta) < DELTA_LIMIT) return current;
        return delta > 0 ? previous + RAMP_UP : previous - RAMP_DOWN;
    }
}
