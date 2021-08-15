package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
//WPI imports
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Globals;
//RobotContainer import
import frc.robot.RobotContainer;
//Subsystem imports
import frc.robot.subsystems.Elevator;

/**
 * SimpleDrive class
 * <p>
 * This class drives a motor
 */
public class MoveElevator extends CommandBase {
    // Grab the subsystem instance from RobotContainer
    private final static Elevator m_elevator = RobotContainer.m_elevator;
    private double dT = 0.02;
    private boolean m_endFlag = false;
    private TrapezoidProfile.Constraints m_constraints;
    private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
    private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();
    public static double distMoved;
    private final double _startSpeed;
    private final double _maxSpeed;
    private final double _endSpeed;
    private double dist;
    private int m_dir;
    private double[] startCo = new double[2];


    /**
     * Constructor
     */
    // This move the robot a certain distance following a trapezoidal speed profile.
    public MoveElevator(double dist, double startSpeed, double endSpeed, double maxSpeed) {
        this.dist = dist;
        _startSpeed = startSpeed;
        _maxSpeed = maxSpeed;
        _endSpeed = endSpeed;
        m_constraints = new TrapezoidProfile.Constraints(_maxSpeed, 1);
        m_setpoint = new TrapezoidProfile.State(0, _startSpeed);

        // Negative distance don't seem to work with the libr ary function????
        // Easier to make distance positive and use m_dir to keep track of negative
        // speed.

        m_dir = (dist>0)?1:-1;
        dist *= m_dir;          
        
        m_goal = new TrapezoidProfile.State(dist, endSpeed);

        //addRequirements(m_arm); // Adds the subsystem to the command

    }

    /**
     * Runs before execute
     */
    @Override
    public void initialize() {
        
        // gets parameters for speed profile
        m_setpoint = new TrapezoidProfile.State(0, _startSpeed);
        m_endFlag = false;
    }
    /**
     * Condition to end speed profile
     */
    public boolean endCondition()
    {
        return false;
        
    }

    /**
     * Called continously until command is ended
     */
        
    @Override
    public void execute()
    {
        
        //Create a new profile to calculate the next setpoint(speed) for the profile
        var profile = new TrapezoidProfile(m_constraints, m_goal, m_setpoint);
        m_setpoint = profile.calculate(dT);
        m_elevator.setElevatorSpeed(m_setpoint.velocity*m_dir);

        if ((m_setpoint.position>=m_goal.position) || endCondition()) {
            //distance reached or end condition met. End the command
            //This class should be modified so that the profile can end on other conditions like
            //sensor value etc.

            m_elevator.setElevatorSpeed(m_goal.velocity*m_dir);
            m_endFlag = true;
        }

        Globals.debug5 = m_goal.position;
        Globals.debug6 = m_setpoint.velocity*m_dir;
        Globals.debug7 = m_setpoint.position;
    }

    /**
     * Called when the command is told to end or is interrupted
     */
    @Override
    public void end(boolean interrupted)
    {
        
    }

    /**
     * Creates an isFinished condition if needed
     */
    @Override
    public boolean isFinished()
    {
        return m_endFlag;
    }


}