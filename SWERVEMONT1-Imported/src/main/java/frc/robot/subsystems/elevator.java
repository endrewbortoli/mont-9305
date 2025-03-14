package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class elevator extends SubsystemBase {

    private final SparkFlex elevatorMotor;
    private SparkClosedLoopController closedLoopController;

    

    public elevator() {
        elevatorMotor = new SparkFlex(Constants.elevatorcontants.kelevatorMotor, MotorType.kBrushless);

        ConfigureMotorController(elevatorMotor, true, IdleMode.kCoast);
        closedLoopController = elevatorMotor.getClosedLoopController();
        SmartDashboard.putNumber("Elevator Target Position", 0);
    }

    public void ConfigureMotorController(SparkFlex Motor, Boolean IsInverted, IdleMode idleMode) {
        SparkFlexConfig MotorConfig = new SparkFlexConfig();

        MotorConfig.softLimit.forwardSoftLimitEnabled(false);
        MotorConfig.softLimit.forwardSoftLimit(Constants.elevatorConstants.KfowarSoftLimit);
        MotorConfig.softLimit.reverseSoftLimitEnabled(false);
        MotorConfig.softLimit.reverseSoftLimit(Constants.elevatorConstants.KreverseSoftLimit);

        /*   PID MAXMOTION*/
        MotorConfig.encoder.positionConversionFactor(1);
        
        MotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .p(Constants.elevatorConstants.kP)
                .i(Constants.elevatorConstants.kI)
                .i(Constants.elevatorConstants.kD)
                .outputRange(-0.5, 0.5);
        
            MotorConfig.closedLoop.maxMotion.maxVelocity(25000).maxAcceleration(50000)
         .allowedClosedLoopError(0.6);

        MotorConfig.inverted(false);
        MotorConfig.idleMode(idleMode);
        MotorConfig.smartCurrentLimit(40);

        Motor.configure(MotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setPosition(double setPoint){
        closedLoopController.setReference(setPoint, ControlType.kMAXMotionPositionControl);
        SmartDashboard.putNumber("Elevator Target Position", setPoint);
    }
    public void StopMotor() {
        elevatorMotor.stopMotor();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Valor encoder Elevator", elevatorMotor.getEncoder().getPosition());
    }
}

