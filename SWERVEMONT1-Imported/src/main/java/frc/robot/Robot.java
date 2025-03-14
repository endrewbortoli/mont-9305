package frc.robot;


import com.revrobotics.spark.SparkMax; // biblioteca do SparkMax
import com.revrobotics.spark.SparkFlex; // biblioteca do SparFlex
import com.revrobotics.spark.SparkLowLevel.MotorType; // biblioteca que define o tipo de motor
import edu.wpi.first.wpilibj.XboxController; // biblioteca do Controle de xbox/import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; (deixa ae, se precisar nós usamos)
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.DigitalInput; //biblioteca sensor
//import edu.wpi.first.wpilibj.Encoder; // biblioteca do encoder 
import edu.wpi.first.wpilibj.TimedRobot; // biblioteca do TimedRobot


/* Clean code chora com meu código */


public class Robot extends TimedRobot {

  private final RobotContainer m_robotContainer;


  //   //private final Sparkflex elevador
  // private final SparkFlex sparkFlex2 = new SparkFlex(11, MotorType.kBrushless); // esquerdo


 public Robot() {
    m_robotContainer = new RobotContainer();
 }

  @Override
  public void teleopPeriodic() {

    

    // //verifica se o joystick está sendo levantado
    // if (xbox1.getAButtonPressed()){ //verifica se o valor do joystick é maior que 0
    //   sparkFlex2.set(-0.10); // Liga o motor esquerdo do elevador a 70% de potência
    // }  else {
    //   sparkFlex2.set(0);
    // }


    // //verifica se o joystick está sendo levantado
    // if (xbox1.getYButtonPressed()){ //verifica se o valor do joystick é maior que 0
    //   sparkFlex2.set(0.05); // Liga o motor esquerdo do elevador a 70% de potência
    // } else {
    //   sparkFlex2.set(0);
    // }



  }

   @Override
public void robotPeriodic(){
      CommandScheduler.getInstance().run();
}

@Override
public void disabledInit() {}

@Override
public void disabledPeriodic() {}

@Override
public void disabledExit() {}

@Override
public void autonomousInit() {
  Command m_autonomousCommand = m_robotContainer.getAutonomousCommand();

  if (m_autonomousCommand != null) {
    m_autonomousCommand.schedule();
  }
}

@Override
public void autonomousPeriodic() {}

@Override
public void autonomousExit() {}


@Override
public void teleopExit() {}

@Override
public void testInit() {
  CommandScheduler.getInstance().cancelAll();
}

@Override
public void testPeriodic() {}

@Override
public void testExit() {}



}





 