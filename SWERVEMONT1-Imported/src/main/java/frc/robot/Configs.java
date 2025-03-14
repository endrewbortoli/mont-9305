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
    public static final SparkMaxConfig outtakeLeftConfig = new SparkMaxConfig();
    public static final SparkMaxConfig outtakeRightConfig = new SparkMaxConfig();

    static {        

      // Configuração do motor de outtake
      outtakeLeftConfig
        .idleMode(IdleMode.kBrake) // Modo de inatividade: Brake
        .smartCurrentLimit(40) // Limite de corrente inteligente: 40A
        .inverted(false);

        outtakeRightConfig
        .idleMode(IdleMode.kBrake) // Modo de inatividade: Brake
        .smartCurrentLimit(40) // Limite de corrente inteligente: 40A
        .inverted(true);
    
}}