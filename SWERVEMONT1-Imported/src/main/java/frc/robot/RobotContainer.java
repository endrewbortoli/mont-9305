// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.Controle;
import frc.robot.commands.AutoDriveForward;
import frc.robot.commands.ElevatorCmd;
import frc.robot.commands.OuttakeCmd;
import frc.robot.subsystems.OuttakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.elevator;

import java.io.File;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
  // Aqui iniciamos o swerve
  private SwerveSubsystem swerve = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
  public static OuttakeSubsystem outtakeSubsystem = new OuttakeSubsystem();
  private final elevator elevatorSubsystem = new elevator();


  
  // Controle de Xbox, troque para o qual sua equipe estará utilizando
  private Joystick DriverJoystick = new Joystick(Controle.DriverJoystick);
  private XboxController OperatorJoystick = new XboxController(Controle.OperatorJoystick);

  public RobotContainer() {

    // Definimos o comando padrão como a tração
    swerve.setDefaultCommand(swerve.driveCommand(
      () -> MathUtil.applyDeadband(DriverJoystick.getRawAxis(1), Constants.Controle.DEADBAND),
      () -> MathUtil.applyDeadband(DriverJoystick.getRawAxis(0), Constants.Controle.DEADBAND),
      () -> DriverJoystick.getRawAxis(4)));

    // Configure the trigger bindings
    configureBindings();
  }

  // Função onde os eventos (triggers) são configurados
  private void configureBindings() {

            // Trigger direito para Outtake
        new Trigger(() -> 
          OperatorJoystick.getRightTriggerAxis() > 0.1)
            .whileTrue(new OuttakeCmd(outtakeSubsystem, "Outtake"));

        // Trigger esquerdo para Intake
        new Trigger(() -> 
          OperatorJoystick.getLeftTriggerAxis() > 0.1)
            .whileTrue(new OuttakeCmd(outtakeSubsystem, "Intake"));

        //             // Posições pré-definidas
        // new JoystickButton(OperatorJoystick, XboxController.Button.kA.value)
        //     .onTrue(new ElevatorCmd(elevatorSubsystem, Constants.ElevatorConstants.kHomePosition));
            
        // new JoystickButton(OperatorJoystick, XboxController.Button.kB.value)
        //     .onTrue(new ElevatorCmd(elevatorSubsystem, Constants.ElevatorConstants.kL2));
            
        // new JoystickButton(OperatorJoystick, XboxController.Button.kY.value)
        //     .onTrue(new ElevatorCmd(elevatorSubsystem, Constants.ElevatorConstants.KL3));

        // new JoystickButton(OperatorJoystick, XboxController.Button.kX.value)
        //     .onTrue(new ElevatorCmd(elevatorSubsystem, Constants.ElevatorConstants.KL4));

    }

  // Função que retorna o autônomo
  public Command getAutonomousCommand() {
    return new AutoDriveForward(swerve, -0.3, 2.0) // 30% de velocidade por 2 segundos
            .withTimeout(2.0); // redundância para segurança
  }

  // Define os motores como coast ou brake
  public void setMotorBrake(boolean brake) {
    swerve.setMotorBrake(brake);
  }
}
