package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.DefaultAcquisition;
import frc.robot.constants.kCANIDs;
import com.revrobotics.CANSparkMax.ControlType;



public class Acquisition extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private final XboxController controller = new XboxController(0);
  double setpointVelocity = 0;
  public CANSparkMax motor2;
  public RelativeEncoder encoder;
  public SparkMaxPIDController pid;
  public Solenoid arms = new Solenoid(31, PneumaticsModuleType.REVPH,0);
  public Acquisition() {
    motor2 = new CANSparkMax(11, MotorType.kBrushless);
    encoder = motor2.getEncoder();


    
    pid = motor2.getPIDController();
  
    pid.setP(5e-5);
    pid.setI(1e-6);
    pid.setD(0);
    pid.setFF(0.000156);
    pid.setIZone(0);
    pid.setOutputRange(-1,1);

    
  
    
  }

  public void extendArms(boolean state) {
    arms.set(state);
  }

  public boolean getArmsExtended(){
    return arms.get();
  }

  public void setVelocity(double setpoint) {
    setpointVelocity=setpoint;
  }

  public double getVelocity(){
    return setpointVelocity;
  }


  
  @Override
  public void periodic() {
    
    if (getArmsExtended()) {
      motor2.set(setpointVelocity);
      SmartDashboard.putNumber("acquisition/actual Velocity", encoder.getVelocity());
      setpointVelocity = SmartDashboard.getNumber("acquisition/setpoint Velocity", setpointVelocity);
      pid.setReference(setpointVelocity, ControlType.kVelocity);
    } else {
      motor2.set(0);
    }


    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  



}