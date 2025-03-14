package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator;

public class ElevatorCmd extends Command {
    private final elevator elevatorSubsystem;
    private final double targetPosition;

    public ElevatorCmd(elevator subsystem, double position) {
        this.elevatorSubsystem = subsystem;
        this.targetPosition = position;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        elevatorSubsystem.setPosition(targetPosition);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}