package frc.robot.subsystems;

import frc.robot.Configs;
import frc.robot.Constants.OuttakeConstants;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;




public class OuttakeSubsystem extends SubsystemBase {
    private static final SparkMax OuttakeLeftMotor = new SparkMax(16, MotorType.kBrushless);
    private static final SparkMax OuttakeRightMotor = new SparkMax(15, MotorType.kBrushless);

      //private final sensor infravermelho da garra do cano
  private final DigitalInput Sensor1 = new DigitalInput(0);

  boolean ValorDoSensor = Sensor1.get(); // O que o robô ve quando o botão A é apertado



    public OuttakeSubsystem() {
        OuttakeLeftMotor.configure(Configs.outtakeLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        OuttakeRightMotor.configure(Configs.outtakeRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }


    @Override
    public void periodic() {
    }

    public void setOuttake(String state) {
        switch (state) {
            case "Intake":
                OuttakeLeftMotor.set(OuttakeConstants.kOuttakeUpMotorMaxSpeed);
                OuttakeRightMotor.set(OuttakeConstants.kOuttakeUpMotorMaxSpeed);
                break;
                
            case "Outtake":
                OuttakeLeftMotor.set(-OuttakeConstants.kOuttakeDownMotorMaxSpeed);
                OuttakeRightMotor.set(-OuttakeConstants.kOuttakeDownMotorMaxSpeed);
                break;
                                
            default: // Parada
                OuttakeLeftMotor.set(0);
                OuttakeRightMotor.set(0);
                break;
        }
    }


        //  // Verifica se o botão B do controle Xbox foi pressionado
        //  if (xbox1.getBButton() && !xbox1.getXButton()) { // O método getAButton() verifica o botão B e X
        //     sparkFlex3.set(-0.20); // Liga o motor 1 com -70% de potência
        //     sparkFlex4.set(-0.20); // Liga o motor 2 com -70% de potência
        //     } else {
        //     sparkFlex3.set(0); // desliga o motor 1 
        //     sparkFlex4.set(0); // desliga o motor 2 
        //   }
      
        //   // Verifica se o botão X do controle Xbox foi pressionado
        //   if (xbox1.getXButton() && !xbox1.getBButton()) { // O método getAButton() verifica o botão X e B
        //   sparkFlex3.set(0.20); // Liga o motor 1 com 70% de potência
        //   sparkFlex4.set(0.20); // Liga o motor 2 com 70% de potência
        //   } else {
        //   sparkFlex3.set(0); // desliga o motor 1 
        //   sparkFlex4.set(0); // desliga o motor 2 
        //   }
      
      

}