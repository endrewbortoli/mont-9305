// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Tracao;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoDriveForward extends Command {
  
  private final SwerveSubsystem swerve;
  private final Timer timer = new Timer();
  private final double duration;
  private final double speedMultiplier;

  public AutoDriveForward(SwerveSubsystem swerve, double speedMultiplier, double durationSeconds) {
    this.swerve = swerve;
    this.speedMultiplier = speedMultiplier;
    this.duration = durationSeconds;
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  @Override
  public void execute() {
    // Cria vetor de translação com velocidade somente no eixo X (frente do robô)
    Translation2d translation = new Translation2d(
        speedMultiplier * Tracao.MAX_SPEED, 
        0
    );
    
    // Comanda o swerve com velocidade angular zero (sem girar)
    swerve.drive(translation, 0, true); // true = field-relative
  }

  @Override
  public void end(boolean interrupted) {
    // Para o robô quando o comando terminar
    swerve.drive(new Translation2d(0, 0), 0, true);
  }

  @Override
  public boolean isFinished() {
    return timer.hasElapsed(duration);
  }
}

