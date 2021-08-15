package frc.robot.commands.auto.PickCommands;

import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
//WPI imports
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Globals;
//RobotContainer import
import frc.robot.RobotContainer;
//Subsystem imports
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Vision;

/**
 * SimpleDrive class
 * <p>
 * This class drives a motor
 */
public class ElevatorPick extends CommandBase {
    // Grab the subsystem instance from RobotContainer
    private final static Vision m_vision = RobotContainer.m_vision;
    private final static Elevator m_elevator = RobotContainer.m_elevator;
    private double dT = 0.02;
    private boolean m_endFlag = false;
    private TrapezoidProfile.Constraints m_constraints;
    private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
    private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();
    public static double distMoved;
    private final double _maxSpeed;
    private double[] startCo = new double[2];
    private double zgoal;
    private int m_dir;

    /**
     * Constructor
     */
    // This move the robot a certain distance following a trapezoidal speed profile.
    public ElevatorPick(double maxSpeed) {

        /*
        item 0 = chips
             1 = ball
             2 = kitkat
             3 = nissin
        */
        
        
        _maxSpeed = maxSpeed;

        m_constraints = new TrapezoidProfile.Constraints(_maxSpeed, 0.5);

        // Negative distance don't seem to work with the libr ary function????
        // Easier to make distance positive and use m_dir to keep track of negative
        // speed.

        //addRequirements(m_arm); // Adds the subsystem to the command

    }

    /**
     * Runs before execute
     */
    @Override
    public void initialize() {
        Globals.debug9++;
        // gets parameters for speed profile
        zgoal = getItemZ(Globals.curItem);

        // startCo = m_arm.getCoordinate(Globals.curAngle1, Globals.curAngle2);
        // dist = m_arm.getDistance(startCo[0], xgoal, startCo[1], ygoal);
        // trajectoryAngle = m_arm.getAngle(startCo[0], xgoal, startCo[1], ygoal);
        m_setpoint = new TrapezoidProfile.State(0, 0);
        m_goal = new TrapezoidProfile.State(zgoal, 0);

        //checks if target coordinates are within boundaries
        
        
        //debug stuff

    }

    public double getItemZ(int item){

        //gets item type to pick and returns item coordinate
        double [] itemCo = new double[4];
        /*
        item 0 = chips
             1 = ball
             2 = kitkat
             3 = nissin
        */
        itemCo[0] = -0.15;
        itemCo[1] = -0.15;
        itemCo[2] = -0.15;
        itemCo[3] = -0.15;  

        // add offset of arm to camera
        return itemCo[item];
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
    }

    /**
     * Called when the command is told to end or is interrupted
     */
    @Override
    public void end(boolean interrupted)
    {
        Globals.debug10++;
        
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