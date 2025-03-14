package frc.robot;


import com.revrobotics.spark.SparkMax; // biblioteca do SparkMax
import com.revrobotics.spark.SparkFlex; // biblioteca do SparFlex
import com.revrobotics.spark.SparkLowLevel.MotorType; // biblioteca que define o tipo de motor
import edu.wpi.first.wpilibj.XboxController; // biblioteca do Controle de xbox/import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; (deixa ae, se precisar nós usamos)
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.DigitalInput; //biblioteca sensor
//import edu.wpi.first.wpilibj.Encoder; // biblioteca do encoder 
import edu.wpi.first.wpilibj.TimedRobot; // biblioteca do TimedRobot


/* Clean code chora com meu código */


public class Robot extends TimedRobot {

  //private final XboxController
  private final XboxController xbox1 = new XboxController(1);

  private final RobotContainer m_robotContainer;


  //private final Sparkmax garra do cano
  private final SparkMax Spark9 = new SparkMax(15, MotorType.kBrushless); // direito
  private final SparkMax Spark10 = new SparkMax(16, MotorType.kBrushless);// esquerdo
  
  //private final Sparkflex garra da bola
  private final SparkFlex sparkFlex3 = new SparkFlex(12, MotorType.kBrushless); // direito
  private final SparkFlex sparkFlex4 = new SparkFlex(13, MotorType.kBrushless); // esquerdo

  //private final sparkmax levantar garra da bola
  //private final SparkMax Spark11 = new SparkMax(14, MotorType.kBrushless);

  //private final Sparkflex elevador
  private final SparkFlex sparkFlex2 = new SparkFlex(11, MotorType.kBrushless); // esquerdo

  //private final Encoder elevador
  //Encoder encoder = new Encoder(0, 1);


  //private final sensor infravermelho da garra do cano
  private final DigitalInput Sensor1 = new DigitalInput(0);

 public Robot() {
    m_robotContainer = new RobotContainer();
 }

  @Override
  public void teleopPeriodic() {


    boolean ValorDoSensorAntes = false; //  O que o sensor viu na ultima vez que apertou A
    boolean ValorDoSensor = Sensor1.get(); // O que o robô ve quando o botão A é apertado

    
    // Verifica se o botão A do controle Xbox foi pressionado
    if (xbox1.getBButton()) { // O método getAButton() verifica o botão A
      Spark9.set(-0.30); // Liga o motor 1 com -70% de potência
      Spark10.set(0.30); // Liga o motor 2 com -70% de potência
    } else {
      Spark9.set(0); // Desliga o motor 1
      Spark10.set(0); // Desliga o motor 2
    }

    if (xbox1.getAButton() && ValorDoSensor == true && ValorDoSensorAntes == false);{ // Se o Botão A do controle Xbox for precionado e o sensor veja algo
      int tempoEmMilissegundos = 500; // 0,5 segundos

      try {
          Thread.sleep(tempoEmMilissegundos); 
      } catch (InterruptedException e) {       // Essa função faz o código pausar pelo tempo determinado
          e.printStackTrace();                 // (Não me pergunte como essa bomba funciona, nem foi testado ainda (lucca mandou mudar isso))
      }

      Spark9.set(0.); // Desliga o motor 1
      Spark10.set(0); // Desliga o motor 2
        
      if (ValorDoSensorAntes == false) {
        ValorDoSensorAntes = true; // Faz a garra poder jogar o cano
      } else {
        ValorDoSensorAntes = false; // Faz a garra segurar o cano
      }
    }

     // Verifica se o botão B do controle Xbox foi pressionado
    if (xbox1.getBButton() && !xbox1.getXButton()) { // O método getAButton() verifica o botão B e X
      sparkFlex3.set(-0.20); // Liga o motor 1 com -70% de potência
      sparkFlex4.set(-0.20); // Liga o motor 2 com -70% de potência
      } else {
      sparkFlex3.set(0); // desliga o motor 1 
      sparkFlex4.set(0); // desliga o motor 2 
    }

    // Verifica se o botão X do controle Xbox foi pressionado
    if (xbox1.getXButton() && !xbox1.getBButton()) { // O método getAButton() verifica o botão X e B
    sparkFlex3.set(0.20); // Liga o motor 1 com 70% de potência
    sparkFlex4.set(0.20); // Liga o motor 2 com 70% de potência
    } else {
    sparkFlex3.set(0); // desliga o motor 1 
    sparkFlex4.set(0); // desliga o motor 2 
    }

    //verifica se o joystick está sendo levantado
    if (xbox1.getAButtonPressed()){ //verifica se o valor do joystick é maior que 0
      sparkFlex2.set(-0.10); // Liga o motor esquerdo do elevador a 70% de potência
    } 

    //verifica se o joystick está sendo levantado
    if (xbox1.getYButtonPressed()){ //verifica se o valor do joystick é maior que 0
      sparkFlex2.set(0.05); // Liga o motor esquerdo do elevador a 70% de potência
    } 

    //verifica se o joystick está sendo levantado
    if (xbox1.getXButtonPressed()){ //verifica se o valor do joystick é maior que 0
       sparkFlex2.set(0); // Liga o motor esquerdo do elevador a 70% de potência
    }

  }

   @Override
public void robotPeriodic(){
      CommandScheduler.getInstance().run();
}

}





 