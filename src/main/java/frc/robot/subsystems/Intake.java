package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;


public class Intake extends SubsystemBase {

  private TimeOfFlight distanceSensor;
  
  //configuration values for pivot motor 
  private static final double k_pivotMotorP =  0.032; //0.01785714;
  private static final double k_pivotMotorI = 0.0;
  private static final double k_pivotMotorD = 0.001;
  private static final double k_pivotMaxOutput = 1;//0.75;
  private static final double k_pivotMinOutput = -1;// -0.75;
  private static final double kIz = 0; 
  private static final double kFF = 0; 
  private double currentPivotPosition = Constants.Intake.k_pivotAngleStow;
  public double maxRPM, maxVel, minVel, maxAcc, allowedErr;
  
  private final SparkAbsoluteEncoder m_AbsolutePivotEncoder; 
  //public final LEDs m_leds = LEDs.getInstance();

  /*-------------------------------- Private instance variables ---------------------------------*/
  private static Intake mInstance;
  private PeriodicIO m_periodicIO;
  private RobotContainer m_RobotContainer;

  /*
  public static Intake getInstance() {
    if (mInstance == null) {
      mInstance = new Intake();
    }
    return mInstance;
  } */

  private CANSparkMax mIntakeMotor;
  private CANSparkMax mPivotMotor;
  private SparkPIDController m_pidController;
  private RelativeEncoder m_encoder;

  public Intake(RobotContainer robotContainer) {

    super("Intake");
    m_RobotContainer = robotContainer;

    mIntakeMotor = new CANSparkMax(Constants.Intake.kIntakeMotorId, MotorType.kBrushless);
    mIntakeMotor.restoreFactoryDefaults();
    mIntakeMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    mIntakeMotor.burnFlash();

    mPivotMotor = new CANSparkMax(Constants.Intake.kPivotMotorId, MotorType.kBrushless);
    mPivotMotor.restoreFactoryDefaults();
    mPivotMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    mPivotMotor.setSmartCurrentLimit(10);

    //mPivotMotor.setSoftLimit(SoftLimitDirection.kReverse, -55);
    //mPivotMotor.setSoftLimit(SoftLimitDirection.kForward, 0);
    //mPivotMotor.enableSoftLimit(SoftLimitDirection.kForward,true );
    //mPivotMotor.enableftLimit(SoftLimitDirection.kReverse,true );
    
    m_AbsolutePivotEncoder = mPivotMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
    
    m_pidController = mPivotMotor.getPIDController();
    m_encoder = mPivotMotor.getEncoder();
    m_encoder.setPosition(0);//m_AbsolutePivotEncoder.getPosition()*-98);

        // set PID coefficients
    m_pidController.setP(k_pivotMotorP);
    m_pidController.setI(k_pivotMotorI);
    m_pidController.setD(k_pivotMotorD);
    m_pidController.setIZone(kIz);
    m_pidController.setFF(kFF);
    m_pidController.setOutputRange(k_pivotMinOutput, k_pivotMaxOutput);
    mPivotMotor.burnFlash();

    m_periodicIO = new PeriodicIO();

    distanceSensor = new TimeOfFlight(0);

  }

  private static class PeriodicIO {
    // Input: Desired state
    PivotTarget pivot_target = PivotTarget.STOW;
    IntakeState intake_state = IntakeState.NONE;

    // Output: Motor set values
    double intake_pivot_voltage = 0.0;
    double intake_speed = 0.0;
  }

  public enum PivotTarget {
    NONE,
    GROUND,
    SOURCE,
    AMP,
    STOW
  }

  public enum IntakeState {
    NONE,
    INTAKE,
    EJECT,
    PULSE,
    FEED_SHOOTER,
  }

  /*-------------------------------- Generic Subsystem Functions --------------------------------*/

  @Override
  public void periodic() {
    checkAutoTasks();

    //Intake position
   m_pidController.setReference(currentPivotPosition, CANSparkMax.ControlType.kPosition);
    
    // Intake control
    m_periodicIO.intake_speed = intakeStateToSpeed(m_periodicIO.intake_state);
    mIntakeMotor.set(m_periodicIO.intake_speed);

    //outputTelemetry();

    //Callback to effect a load.  
    if(getIntakeHasNote() && isPivotStowed())
      m_RobotContainer.scheduleLoadingCommand();
  }

  public void testMotor(boolean on)
  {
       mIntakeMotor.set(0.25);
  }


  /*@Override
  public void writePeriodicOutputs() {}*/

  //@Override
  public void stop() {

    mPivotMotor.stopMotor();

    m_periodicIO.intake_pivot_voltage = 0.0;
    m_periodicIO.intake_speed = 0.0;
  }

 // @Override
  public void outputTelemetry() {
    /*putNumber("Encoder position",m_encoder.getPosition()) ;
    putNumber("Distance  position",distanceSensor.getRange() ) ;
    putNumber("Pivot Current", mPivotMotor.getOutputCurrent());
    putNumber("Pivot Speed",m_encoder.getVelocity());
    putNumber("Pivot movement status",Math.abs(m_encoder.getPosition() - Constants.Intake.k_pivotAngleStow));*/
  }

  //@Override
  public void reset() {
  }

  public double intakeStateToSpeed(IntakeState state) {
    switch (state) {
      case INTAKE:
        return Constants.Intake.k_intakeSpeed;
      case EJECT:
        return Constants.Intake.k_ejectSpeed;
      case PULSE:
        // Use the timer to pulse the intake on for a 1/16 second,
        // then off for a 15/16 second
        if (Timer.getFPGATimestamp() % 1.0 < (1.0 / 45.0)) {
          return Constants.Intake.k_intakeSpeed;
        }
        return 0.0;
      case FEED_SHOOTER:
        return Constants.Intake.k_feedShooterSpeed;
      default:
        // "Safe" default
        return 0.0;
    }
  }

  /*---------------------------------- Custom Public Functions ----------------------------------*/

  public IntakeState getIntakeState() {
    return m_periodicIO.intake_state;
  }

  public boolean getIntakeHasNote() {
 
    if(distanceSensor.getRange() < 200)
      return true;
    else 
      return false;
  }

  // Pivot helper functions
  public void goToGround() {
    
    if(getIntakeHasNote())  //Don't do anything if there's already a note. 
      return;

    currentPivotPosition = Constants.Intake.k_pivotAngleGround;

    m_periodicIO.pivot_target = PivotTarget.GROUND;
    m_periodicIO.intake_state = IntakeState.INTAKE; 
    //m_leds.setColor(Color.kYellow);
  }

  public void goToSource() {

    m_periodicIO.pivot_target = PivotTarget.SOURCE;
    m_periodicIO.intake_state = IntakeState.NONE;
  }

  public void goToAmp() {
    currentPivotPosition = Constants.Intake.k_pivotAngleAmp;
    m_periodicIO.pivot_target = PivotTarget.SOURCE;
    m_periodicIO.intake_state = IntakeState.NONE;
  }

  public void goToStow() {
  currentPivotPosition = Constants.Intake.k_pivotAngleStow;
    m_periodicIO.pivot_target = PivotTarget.STOW;
    m_periodicIO.intake_state = IntakeState.NONE;
  }

  // Intake helper functions
  public void intake() {
    m_periodicIO.intake_state = IntakeState.INTAKE;
  }

  public void eject() {
    m_periodicIO.intake_state = IntakeState.EJECT;
  }

  public void pulse() {
    m_periodicIO.intake_state = IntakeState.PULSE;
  }

  public void feedShooter() {
    m_periodicIO.intake_state = IntakeState.FEED_SHOOTER;
  }

  public void stopIntake() {
    m_periodicIO.intake_state = IntakeState.NONE;
    m_periodicIO.intake_speed = 0.0;
  }

  public void setState(IntakeState state) {
    m_periodicIO.intake_state = state;
  }

  public void setPivotTarget(PivotTarget target) {
    m_periodicIO.pivot_target = target;
  }

  /*---------------------------------- Custom Private Functions ---------------------------------*/
  private void checkAutoTasks() {
    // If the intake is set to GROUND, and the intake has a note, and the pivot is
    // close to it's target
    // Stop the intake and go to the SOURCE position

    if (m_periodicIO.pivot_target == PivotTarget.GROUND && getIntakeHasNote()){//) && isPivotAtTarget()) {
      m_periodicIO.pivot_target = PivotTarget.STOW;
      m_periodicIO.intake_state = IntakeState.NONE;
      currentPivotPosition = Constants.Intake.k_pivotAngleStow;
      //m_leds.setColor(Color.kGreen);
    }
  }

  public void StopAllMotors ()
  {
    m_periodicIO.intake_state = IntakeState.NONE;
    mPivotMotor.set(0);
  }

  public void StopIntakeMotor()
  {
    mIntakeMotor.set(0.0);
    m_periodicIO.intake_state = IntakeState.NONE; 
  }
  public void ReverseIntake(){

    m_periodicIO.intake_state = IntakeState.EJECT; 
  }
  private boolean isPivotAtTarget() {
    return Math.abs(getPivotAngleDegrees() - pivotTargetToAngle(m_periodicIO.pivot_target)) < 5;
  }
  
  public boolean isPivotStowed() {
      return Math.abs(m_encoder.getPosition() - Constants.Intake.k_pivotAngleStow) < 5;
  }

//******************************deprecated */
  public double getPivotAngleDegrees() {
    /*double value = m_pivotEncoder.getPosition(); /*- Constants.Intake.k_pivotEncoderOffset + 0.5;

    return Units.rotationsToDegrees(Helpers.modRotations(value));*/
    return 0.0;
  }

   public double pivotTargetToAngle(PivotTarget target) {
    switch (target) {
      case GROUND:
        return Constants.Intake.k_pivotAngleGround;
      case SOURCE:
        return Constants.Intake.k_pivotAngleSource;
      case AMP:
        return Constants.Intake.k_pivotAngleAmp;
      case STOW:
        return Constants.Intake.k_pivotAngleStow;
      default:
        // "Safe" default
        return 180;
    }
  } 
}