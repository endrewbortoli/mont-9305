// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;

  /**
   * Contém as configurações para os motores do robô.
   */
  public final class Configs {
    /** Configuração para o motor de outtake. */
    public static final SparkMaxConfig outtakeConfig = new SparkMaxConfig();

    /** Configuração para o motor de wrist. */
    public static final SparkMaxConfig wristConfig = new SparkMaxConfig();
    
    /** Configuração para o motor de joint. */
    public static final SparkMaxConfig jointConfig = new SparkMaxConfig();
    
    /** Configuração para o motor de elevator. */
    public static final SparkMaxConfig elevatorConfig = new SparkMaxConfig();
    
    /** Configuração para o motor de driving. */
    public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();

    /** Configuração para o motor de turning. */
    public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

    static {        

      // Configuração do motor de outtake
      outtakeConfig
        .idleMode(IdleMode.kBrake) // Modo de inatividade: Brake
        .smartCurrentLimit(40) // Limite de corrente inteligente: 40A
        .inverted(true);
    
      // Configuração do motor de wrist
      wristConfig
        .idleMode(IdleMode.kBrake) // Modo de inatividade: Brake
        .smartCurrentLimit(40) // Limite de corrente inteligente: 40A
        .inverted(true);
    
      // Configuração do motor de joint
      jointConfig
        .idleMode(IdleMode.kBrake) // Modo de inatividade: Brake
        .smartCurrentLimit(40) // Limite de corrente inteligente: 40A
        .inverted(false);

      // Configuração do motor de elevator
      elevatorConfig
        .idleMode(IdleMode.kBrake) // Modo de inatividade: Brake
        .smartCurrentLimit(80) // Limite de corrente inteligente: 60A
        .inverted(true);
      
      elevatorConfig
        .limitSwitch
          .reverseLimitSwitchEnabled(true)
          .reverseLimitSwitchType(Type.kNormallyOpen);
      
      elevatorConfig
        .closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          .pid(0.04, 0, 0)
          .outputRange(-1, 1)
        .maxMotion
          // Set MAXMotion parameters for position control
          // Define os parâmetros MAXMotion para controle de posição
          .maxVelocity(4200)
          .maxAcceleration(6000)
          .allowedClosedLoopError(0.5);
        
}}