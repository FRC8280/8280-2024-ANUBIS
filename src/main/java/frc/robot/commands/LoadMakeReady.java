package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class LoadMakeReady extends Command {
    
    public Shooter m_Shooter;
    public Intake m_Intake;
    public LoadingModes m_Mode;
    private Timer m_PauseTimer;
    private boolean abort;
    private Timer m_EmergencyTimer;

    public enum LoadingModes {
        WAIT_TIMER,
        LOAD_NOTE,
        ADJUST_NOTE,
      }
    
    public LoadMakeReady(Shooter shooter,Intake intake)
    {
        m_Shooter = shooter;
        m_Intake = intake;
        m_PauseTimer = new Timer();
    }

    @Override public void initialize()
    {
        m_EmergencyTimer = new Timer();
        m_EmergencyTimer.start();

        if(m_Intake.getIntakeHasNote() && m_Intake.isPivotStowed())
            abort = false;
        else
        {
            if(!m_Intake.isPivotStowed())
                System.out.printf("******Abording load and make ready. Pivot isn't stowed******\n");
            else
                System.out.printf("******No Note******\n");
            abort = true;
            return;
        }

        m_PauseTimer.reset();
        m_PauseTimer.start();
        m_Mode = LoadingModes.WAIT_TIMER;
    }

    @Override
    public void execute() 
    {
        if(abort)   //Don't execute if we're aborting. 
            return;

        if(m_Mode == LoadingModes.WAIT_TIMER)
        {
            if(m_PauseTimer.hasElapsed(0.1))
            {
                m_Shooter.IntakeIndexer();
                m_Intake.ReverseIntake();
                m_Mode = LoadingModes.LOAD_NOTE;
            }
        }
        if(m_Mode == LoadingModes.LOAD_NOTE)
        {
            if(m_Shooter.getShooterHasNote())
            {
                m_Mode = LoadingModes.ADJUST_NOTE;
                
                m_Intake.StopIntakeMotor();
                m_Shooter.AdjustIndexPostIntake();
            }
        }   

    }

    @Override public void end(boolean interrupted)
    {
        m_Intake.StopIntakeMotor();
        m_Shooter.StopIndexMotor();
    }
    
    @Override public boolean isFinished()
    {   
        if(abort)
            return true;

        //Emergency Timer
        if(m_EmergencyTimer.hasElapsed(3))
        {
             //System.out.printf("******Load command emergency timer******\n");
           return true; 
        }
    
        if(m_Mode == LoadingModes.ADJUST_NOTE)
        {
            if(m_Shooter.getShooterIsAdjusted())
            {
                m_Shooter.StopIndexMotor();
                return true;
            }
        }

        return false;
    }
}
