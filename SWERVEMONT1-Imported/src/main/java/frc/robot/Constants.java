// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.geometry.Translation3d;
import swervelib.math.Matter;
import swervelib.parser.PIDFConfig;

/**
 * Classe de constantes
 */
public final class Constants {
  // Aqui temos várias constantes referentes as demais áreas do robô
    
  public static final class Dimensoes {
    // Tempo de loop (sparkMax + normal = 130ms)
    public static final double LOOP_TIME = 0.13;
    // Massa do robô
    public static final double ROBOT_MASS = 38;
    //Velocidade máxima
    public static final double MAX_SPEED = 4;

    //Posições do centro de massa
    private static final double xMass = 0;
    private static final double yMass = 0;
    private static final double zMass = .08;

    // Centro de massa do chassi
    public static final Matter CHASSIS    = new Matter(new Translation3d(xMass, yMass, (zMass)), ROBOT_MASS);
   }

    // Classe que contém os PID para o autônomo
    public static final class PID {
      // PID para frente e para trás
      public static final PIDFConfig xAutoPID     = new PIDFConfig(0.65, 0, 0.05);
      // PID para esquerda e direita
      public static final PIDFConfig yAutoPID     = new PIDFConfig(0.6, 0, 0);
      // PID de rotação
      public static final PIDFConfig angleAutoPID = new PIDFConfig(0.1, 0, 0.01);
    }

    // Contem a porta em que o controle está
    public static final class Controle {
      // Porta do controle
      public static final int DriverJoystick = 0;
      public static final int OperatorJoystick = 0;
      
      // Deadband do controle
      public static final double DEADBAND = 0.0;
    }

    public static final class Tracao {
      // Define se a tração vai ser orientada ao campo (sim = true)
      public static final boolean fieldRelative = true;
      // false para malha-fechada
      public static final boolean isOpenLoop = false;
      // true para correção de aceleração
      public static final boolean accelCorrection = false;
      // constante para diminuir o input do joystick (0 < multiplicadorRotacional <= 1)
      public static final double multiplicadorRotacional = 0.3;
      // constante para diminuir o input do joystick (y)
      public static final double multiplicadorTranslacionalY = 0.5;
      // constante para diminuir o input do joystick (x)
      public static final double multiplicadorTranslacionalX = 0.5;

      public static final double TURN_CONSTANT = 0.75;

      // constante que define a velocidade máxima  (era 4.4)
      public static final double MAX_SPEED = 2.0;

      public static final double dt = 0.02;

      public static final double constantRotation = 3;
    }

    public static final class elevatorConstants{

      public static final double KfowarSoftLimit = 0;
    public static final double KreverseSoftLimit = 0;
    public static final double kP = 0.2;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final int kelevatorMotor = 11; //Id elevador 11

    }

    public static class OuttakeConstants {
      /** ID do motor de outtake */
  //private final Sparkmax garra do cano
  private final SparkMax OuttakeMotorIdDireito = new SparkMax(15, MotorType.kBrushless); // direito
  private final SparkMax OuttakeMotorIdEsquerdo = new SparkMax(16, MotorType.kBrushless);// esquerdo
   
      /** Constantes PID para controle de angulação */
      public static final double kPIDAngulationMotorKp = 0.04;
      public static final double kPIDAngulationMotorKi = 0;
      public static final double kPIDAngulationMotorKd = 0;

      /** Posição de referência para o motor de outtake */
      public static double kOuttakeMotorSetPoint = 0;

      public static final double kOuttakeUpMotorMaxPosition = 1.5;
      public static final double kOuttakeUpMotorMinPosition = 0.5;

      /** Velocidade máxima do motor para mover o outtake para cima */
      public static final double kOuttakeUpMotorMaxSpeed = 0.10;
      /** Velocidade máxima do motor para mover o outtake para baixo */
      public static final double kOuttakeDownMotorMaxSpeed = -0.10;
    }


    public final class ElevatorConstants {
      public static final int kelevatorMotor = 14;
      
      // Posições pré-definidas (em unidades do encoder)
      public static final double kHomePosition = 0;
      public static final double kL2 = 50;
      public static final double KL3 = 100;
      public static final double KL4 = 125;
      
      // Soft Limits
      public static final double KfowarSoftLimit = 120;
      public static final double KreverseSoftLimit = -10;
      
      // PID
      public static final double kP = 0.1;
      public static final double kI = 0.0;
      public static final double kD = 0;
  

  }}
   
    
   





