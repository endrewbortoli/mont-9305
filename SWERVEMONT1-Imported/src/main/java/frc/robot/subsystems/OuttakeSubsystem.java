package frc.robot.subsystems;

import frc.robot.Constants.OuttakeConstants;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class OuttakeSubsystem extends SubsystemBase {
    private final SparkMax OuttakeLeftMotor = new SparkMax(16, MotorType.kBrushless);
    private final SparkMax OuttakeRightMotor = new SparkMax(15, MotorType.kBrushless);

    public OuttakeSubsystem() {
        OuttakeLeftMotor.setIdleMode(IdleMode.kCoast);
        OuttakeRightMotor.setIdleMode(IdleMode.kCoast);
        
        OuttakeLeftMotor.setSmartCurrentLimit(50);
        OuttakeRightMotor.setSmartCurrentLimit(50);
        
        // Inverte a direção de um dos motores se necessário
        OuttakeRightMotor.setInverted(true);
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
}