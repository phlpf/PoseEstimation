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
  private double setpointRPM = 0;
  private CANSparkMax motor;
  private RelativeEncoder encoder;
  private SparkMaxPIDController pid;
  private Solenoid arms = new Solenoid(kCANIDs.PNEUMATIC_HUB, PneumaticsModuleType.REVPH, kPneumatics.ACQ_ARMS);
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

  /*public void setArmsExtended(boolean isExtended) {
    arms.set(isExtended);}*/
    /*public boolean getArmsExtended(){
    return arms.get();
  }*/
  
  
  
    public void extendArms(){
    arms.set(true);
  }
  public void retractArms(){
    arms.set(false);
  }
  public boolean areArmsExtended(){
    return arms.get();
  }

  public void stopRollersByVoltage(){
    motor.setVoltage(0);
  }
  
  public void runClosedLoopRPM(){
    pid.setReference(setpointRPM, ControlType.kVelocity);
  }

  public double getSetpointRPM(){
    return setpointRPM;
  }
  public void setRollerRPM(double setpoint) {
    this.setpointRPM = setpoint;
  }
  public double getRollerRPM(){
    return encoder.getVelocity();
  }


  
  @Override
  public void periodic() {
    SmartDashboard.putNumber("A-Acq", motor.getOutputCurrent());
    SmartDashboard.putNumber("Acq-RPM", getRollerRPM());
    SmartDashboard.putNumber("Acq-setpointRPM", getSetpointRPM());
    SmartDashboard.putBoolean("Acq-armsOut", areArmsExtended());//make it green or red box
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  



}