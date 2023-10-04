package frc.robot.subsystems;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.*;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import static frc.robot.Constants.*;


public class ArmSubsystem extends SubsystemBase {
    //defining the spark max arm motor
    private CANSparkMax m_armLeader;
    private CANSparkMax m_armFollower;
    
    private MotorControllerGroup m_MotorControllerGroup;
    //arm speed scale factor
    
    
    //Defineing the arm encoder
    public RelativeEncoder m_lArmEncoder;
    public RelativeEncoder m_fArmEncoder;
    public double m_groupArmEncoder;
    public double one = m_lArmEncoder.getPosition() + 2;
    //m_fArmEncoder.getPosition();
    
   
    public ArmSubsystem() {
        m_armLeader = new CANSparkMax(ArmConstants.kArmLeader, MotorType.kBrushless);
        m_armFollower = new CANSparkMax(ArmConstants.kArmFollower, MotorType.kBrushless);
        m_armFollower.setInverted(true);
        m_MotorControllerGroup = new MotorControllerGroup(m_armLeader, m_armFollower);

        // Need  value for Arm Encoder in Constents
        m_lArmEncoder = m_armLeader.getEncoder();
        m_fArmEncoder = m_armFollower.getEncoder();
        m_lArmEncoder.setPosition(0);
        m_fArmEncoder.setPosition(0);
        
        //m_armMotor.setSmartCurrentLimit(30, 90, 10);
    }
    
     /**
   * This command resets the arm Encoders
   * 
   * @return a runOnce
   */
    public CommandBase resetEncoders() {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        return runOnce(
            () -> {
                m_lArmEncoder.setPosition(0);
                m_fArmEncoder.setPosition(0);
            });
      }
    

  /**
   * Makes sure that the motor will run with the imput you have given.
   * @param forwardSpeed
   * @return the constrainted forwardSpeed
   */
  private double applyLinearConstraints(double forwardSpeed) {
    double result = forwardSpeed;
    double absSpeed = Math.abs(forwardSpeed);

    if (absSpeed < ArmConstants.kDeadzone) {
      result = 0.0;
    }
    else if (absSpeed < ArmConstants.kMinNeededToMove) {
      result = 0.1 * (forwardSpeed/Math.abs(forwardSpeed));
    }
    else if (absSpeed > ArmConstants.kSpeedLimit)
    {
      result = ArmConstants.kSpeedLimit * (forwardSpeed/Math.abs(forwardSpeed));
    }

    return result;
  }

  /**
   * ArmGo applies linear constraints which is a method that
   * "does math to limit the motor and make sure you don't screw up"
   * to the imputed forwardSpeed variable.
   * It then sets m_armMotor to this speed.
   * @param forwardSpeed
   **/
  public void ArmGo(double forwardSpeed) {


    forwardSpeed = applyLinearConstraints(forwardSpeed);

    m_MotorControllerGroup.set(forwardSpeed);
  }

  // public void ArmGoEncoder(double speed) {

  //   speed = applyLinearConstraints(speed);
  //   if ( m_armEncoder.getPosition() < 10000) {
  //      m_armMotor.set(speed);}
  //   else
  //     m_armMotor.set(0);
  // }
  
  /**
   * Sets m_armMotor speed to zero
   */
  public void stop() {

    m_MotorControllerGroup.set(0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Arm Motor Speed", m_armLeader.get());
    
  }


  // public Command ArmForward(double speed) {
  //   return startEnd(() -> {this.m_armMotor.set(0.5);}, () -> {m_armMotor.set(0.0);});

  // }

  // public Command ArmBackward(double speed) {
  //   return startEnd(() -> {this.m_armMotor.set(-0.5);}, () -> {m_armMotor.set(0.0);});
  // }


//   public CommandBase ArmGo(double speed) {
//     return run(
//       () -> {
//         double armSpeed = speed;
//         if (armSpeed < -1) {
//           armSpeed = -1;
//         }
//         else if (armSpeed > 1){
//           armSpeed = 1;
//         }
//         m_armMotor.set(speed * kScaleFactor);
//         SmartDashboard.putNumber("Arm Speed", speed);
//       });
//   };
}