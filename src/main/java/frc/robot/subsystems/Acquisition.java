package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.kCANIDs;
import frc.robot.constants.kPneumatics;



public class Acquisition extends SubsystemBase {
  double setpointVelocity = 0;
  public CANSparkMax motor;
  public RelativeEncoder encoder;
  public SparkMaxPIDController pid;
  public Solenoid arms = new Solenoid(kCANIDs.PNEUMATIC_HUB, PneumaticsModuleType.REVPH, kPneumatics.ACQ_ARMS);
  public Acquisition() {
    motor = new CANSparkMax(kCANIDs.ACQ_MOTOR, MotorType.kBrushless);
    motor.restoreFactoryDefaults();
    motor.setIdleMode(IdleMode.kCoast);

    encoder = motor.getEncoder();
    
    pid = motor.getPIDController();
    pid.setP(5e-5);
    pid.setI(1e-6);
    pid.setD(0);
    pid.setFF(0.000156);
    pid.setIZone(0);
    pid.setOutputRange(-1,1);    
  }

  public void setArmsExtended(boolean isExtended) {
    arms.set(isExtended);
  }

  public boolean getArmsExtended(){
    return arms.get();
  }

  public void setRollerVelocity(double setpoint) {
    this.setpointVelocity = setpoint;
  }

  public double getRollerVelocity(){
    return setpointVelocity;
  }


  
  @Override
  public void periodic() {
    
    if (getArmsExtended()) {
      SmartDashboard.putNumber("acquisition/actual Velocity", encoder.getVelocity());
      setpointVelocity = SmartDashboard.getNumber("acquisition/setpoint Velocity", setpointVelocity);
      pid.setReference(setpointVelocity, ControlType.kVelocity);
    } else {
      motor.set(0);
    }

    SmartDashboard.putNumber("A-Acq", motor.getOutputCurrent());
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  



}