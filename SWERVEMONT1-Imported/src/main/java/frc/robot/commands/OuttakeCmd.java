package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.OuttakeSubsystem;

public class OuttakeCmd extends Command {
    private final OuttakeSubsystem outtakeSubsystem;
    private String state;

    public OuttakeCmd(OuttakeSubsystem outtakeSubsystem, String state) {
        this.outtakeSubsystem = outtakeSubsystem;
        this.state = state;
        addRequirements(outtakeSubsystem);
    }

    @Override
    public void initialize() {
        outtakeSubsystem.setOuttake(state);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}