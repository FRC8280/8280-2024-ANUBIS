package frc.robot.subsystems;

import com.playingwithfusion.TimeOfFlight;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkFlex;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.Intake.PivotTarget;

public class Shooter extends SubsystemBase {

  private CANSparkFlex m_topMotor;
  private CANSparkFlex m_bottomMotor;
  private CANSparkMax m_indexMotor;
  private CANSparkMax m_sPivotMotor;

  private SparkPIDController m_PidController;
  private RelativeEncoder m_topMotorEncoder;
  private RelativeEncoder m_bottomMotorEncoder;

  private RelativeEncoder mEncoder;
  private final SparkAbsoluteEncoder m_pAbsoluteEncoder; 

  private SparkPIDController m_TopPidController;
  private SparkPIDController m_BottomPidController;

  private boolean m_pivotRunning;
  private boolean m_shooterRunning;
  public double m_shooterMotorSpeed = 0;

  private static final double k_sPivotMotorP =  0.5;
  private static final double k_sPivotMotorI = 0.0;
  private static final double k_sPivotMotorD = 0.001;
  private static final double k_sPivotMaxOutput = 0.25;
  private static final double k_sPivotMinOutput = -0.25;
  private static final double kIz = 0; 
  private static final double kFF = 0; 
  private double currentSPivotPosition = Constants.Shooter.k_ShooterPivotLoaded;


  //Shooter PID
  private static final double k_sShooterMotorP =  0.00006;
  private static final double k_sShooterMotorI = 0.0;
  private static final double k_sShooterMotorD = 0.000;
  private static final double k_sShooterMaxOutput = 1;
  private static final double k_sShooterMinOutput = -1;
  private static final double kShooterIz = 0; 
  private static final double kShooterFF = 0.000159; 

  private TimeOfFlight distanceSensor;

  /** Creates a new ShooterSubsystem. */
  public Shooter() {
    // create two new SPARK MAXs and configure them
    
    m_sPivotMotor =
      new CANSparkMax(Constants.Shooter.kPivotShooterCanId, CANSparkLowLevel.MotorType.kBrushless);
    m_sPivotMotor.setInverted(false);
    m_sPivotMotor.setSmartCurrentLimit(Constants.Shooter.kSPivotCurrentLimit);
    m_sPivotMotor.setIdleMode(IdleMode.kCoast);
    m_sPivotMotor.burnFlash();

    m_PidController = m_sPivotMotor.getPIDController();
    m_PidController.setP(k_sPivotMotorP);
    m_PidController.setI(k_sPivotMotorI);
    m_PidController.setD(k_sPivotMotorD);
    m_PidController.setIZone(kIz);
    m_PidController.setFF(kFF);
    m_PidController.setOutputRange(k_sPivotMinOutput, k_sPivotMaxOutput);

    m_pAbsoluteEncoder = m_sPivotMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
    mEncoder = m_sPivotMotor.getEncoder();//(SparkRelativeEncoder.Type.kHallSensor, 42);
    mEncoder.setPosition(m_pAbsoluteEncoder.getPosition()* 48.6229508);  //seto to absolute encoder value * constant

    m_topMotor =
        new CANSparkFlex(Constants.Shooter.kTopCanId, CANSparkLowLevel.MotorType.kBrushless);
    m_topMotor.setInverted(false);
    m_topMotor.setSmartCurrentLimit(Constants.Shooter.kCurrentLimit);
    m_topMotor.setIdleMode(IdleMode.kCoast);
    m_topMotorEncoder = m_topMotor.getEncoder();

    m_bottomMotor =
        new CANSparkFlex(Constants.Shooter.kBottomCanId, CANSparkLowLevel.MotorType.kBrushless);
    m_bottomMotor.setInverted(false);
    m_bottomMotor.setSmartCurrentLimit(Constants.Shooter.kCurrentLimit);
    m_bottomMotor.setIdleMode(IdleMode.kCoast);
    m_bottomMotorEncoder = m_bottomMotor.getEncoder();

    m_indexMotor =
      new CANSparkMax(Constants.Shooter.kIndexCanId, CANSparkLowLevel.MotorType.kBrushless);
    m_indexMotor.setInverted(false);
    m_indexMotor.setSmartCurrentLimit(Constants.Shooter.kIndexCurrentLimit);
    m_indexMotor.setIdleMode(IdleMode.kBrake);
    m_indexMotor.burnFlash();

    m_TopPidController = m_topMotor.getPIDController();
    m_TopPidController.setP(k_sShooterMotorP);
    m_TopPidController.setI(k_sShooterMotorI);
    m_TopPidController.setD(k_sShooterMotorD);
    m_TopPidController.setIZone(kShooterIz);
    m_TopPidController.setFF(kShooterFF);
    m_TopPidController.setOutputRange(k_sShooterMinOutput, k_sShooterMaxOutput);

    m_BottomPidController = m_bottomMotor.getPIDController();
    m_BottomPidController.setP(k_sShooterMotorP);
    m_BottomPidController.setI(k_sShooterMotorI);
    m_BottomPidController.setD(k_sShooterMotorD);
    m_BottomPidController.setIZone(kShooterIz);
    m_BottomPidController.setFF(kShooterFF);
    m_BottomPidController.setOutputRange(k_sShooterMinOutput, k_sShooterMaxOutput);

    m_topMotor.burnFlash();
    m_bottomMotor.burnFlash();

    m_pivotRunning = false;
    m_shooterRunning = false;
    distanceSensor = new TimeOfFlight(40);
  }

  /**
   * Turns the shooter on. Can be run once and the shooter will stay running or run continuously
   * in a {@code RunCommand}.
   */
  public void runShooter() {
    m_shooterRunning = true;
  }

  /**
   * Turns the shooter off. Can be run once and the shooter will stay running or run continuously
   * in a {@code RunCommand}.
   */
  public void stopShooter() {
    m_shooterRunning = false;
  }


  public void runManual(double position)
  {
      //Todo add implementations
  }

  public void IncreasePivot(double increment)
  {
      System.out.printf("******Increase Pivot****** %f + %f\n",currentSPivotPosition,increment);

      if(currentSPivotPosition + increment < 34 )
        currentSPivotPosition+= increment;
  }

  public void DecreasePivot(double increment)
  {
      System.out.printf("******Decrease Pivot****** %f + %f\n",currentSPivotPosition,increment);
      if(currentSPivotPosition - increment > 1 )
        currentSPivotPosition -= increment;
  }
  //manual pivot commands to test launcher

  public void SubWooferShot()
  {
     currentSPivotPosition = Constants.Shooter.k_SubWoofer;  
  }
  public void pivotZero()
  {
      currentSPivotPosition = Constants.Shooter.k_PivotZero;  
  }

  public void pivotLoaded()
  {
    currentSPivotPosition = Constants.Shooter.k_ShooterPivotLoaded;
  }

  public void pivotFortyFive()
  {
      currentSPivotPosition = Constants.Shooter.k_PivotMiddle;
  }

  public void pivotNinety()
  {
    currentSPivotPosition = Constants.Shooter.k_PivotNinety;
  }

  public enum PivotTarget {
    NONE,
    LOW,
    MEDIUM,
    HIGH,
    LOADED
  }

  public boolean getShooterHasNote() {

    if(distanceSensor.getRange() < Constants.Shooter.kNoteDetectionDistance)
      return true;
    else 
      return false;
  }

  public boolean getShooterIsAdjusted() {

    if(distanceSensor.getRange() > 90)
    {
      //System.out.printf("******Adjusted value over 70******\n");
      return true;
    }
    else 
      return false;
  }

  public void SetShooterAngle(double angle)
  {
    currentSPivotPosition = angle;
  }
  public void SpinShootingMotors(double motorspeed)  //Fires a shot at max velocity without changing angle
  {
      m_shooterRunning = true;
      m_shooterMotorSpeed = motorspeed;//Constants.Shooter.k_ShooterRPM;
  }

  public void StopAllMotors()
  {
    m_sPivotMotor.set(0);
    m_indexMotor.set(0.0);
    m_topMotor.set(0.0);
    m_bottomMotor.set(0.0);
  }

  public void AdjustIndexPostIntake()
  {
    m_indexMotor.set(-0.10);
  }
  public void IntakeIndexer()
  {
    m_indexMotor.set(Constants.Shooter.kIndexPower);
  }
  public void StartIndexMotor()
  {
    m_indexMotor.set(Constants.Shooter.kIndexPower);
  }

  public void StopAllShooterMotors()
  {
    StopIndexMotor();
    StopShootingMotor();
  }

  public void StopIndexMotor()
  {
    m_indexMotor.set(0);
  }
  public void StopShootingMotor()
  {
    m_shooterRunning = false;
    m_shooterMotorSpeed = 0;
  }

  public Boolean AtTargetPivot()
  {
    if(java.lang.Math.abs(mEncoder.getPosition() - currentSPivotPosition) > Constants.Shooter.k_PivotMargin)
      return false;

    return true;

  }
  public Boolean AtTargetSpeed()  //Returns True if the motors have reached the velocity
  {
      if(  (m_topMotorEncoder.getVelocity() / m_shooterMotorSpeed ) < 0.9   )
        return false;
      if(   (m_bottomMotorEncoder.getVelocity() / m_shooterMotorSpeed ) < 0.9 )
        return false;

      return true;
  }
  @Override
  public void periodic() { 
    
     m_PidController.setReference(currentSPivotPosition, CANSparkMax.ControlType.kPosition);

    m_TopPidController.setReference(m_shooterMotorSpeed, CANSparkFlex.ControlType.kVelocity);
    m_BottomPidController.setReference(m_shooterMotorSpeed, CANSparkFlex.ControlType.kVelocity);

    SmartDashboard.putNumber("Sh/Top Motor Speed", m_topMotorEncoder.getVelocity());
    SmartDashboard.putNumber("Sh/Bottom Motor Speed", m_bottomMotorEncoder.getVelocity());
    SmartDashboard.putNumber("Sh/Pivot Position", mEncoder.getPosition());
    SmartDashboard.putNumber("Shooter Distance", distanceSensor.getRange());
    SmartDashboard.putNumber("Set Shooter Motor Speed", m_shooterMotorSpeed);
    }
  }
