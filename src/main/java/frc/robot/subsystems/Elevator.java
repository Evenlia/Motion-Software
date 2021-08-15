package frc.robot.subsystems;

//Java imports

//Vendor imports
import com.kauailabs.navx.frc.AHRS;
import com.studica.frc.TitanQuad;
import com.studica.frc.TitanQuadEncoder;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
//import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
//WPI imports
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Globals;

public class Elevator extends SubsystemBase
{
    //Creates all necessary hardware interface here for omni-drive

    //Motors and encoders
    private final TitanQuad ele_motors;
    private final TitanQuadEncoder ele_encoders;
    //vmx private final Encoder[] encoders;  //VMX encoder

    //PID stuff
    private PIDController pidControllers;
    private double pidInput;
    private double pidOutput;
    private double encoderDists;
    private double encoderSpeeds;

    //Limit Switch
    public DigitalInput limitSwch;

    //For testing. These should be in another subsystem
    private double pid_dT = Constants.PID_DT;

    // Shuffleboard
    private final ShuffleboardTab tabele = Shuffleboard.getTab("Elevator");
    private final NetworkTableEntry D_encoderDisp3 = tabele.add("Encoder3", 0).getEntry();
    private final NetworkTableEntry D_inputSwch = tabele.add("inputSwch", true).getEntry();
    private final NetworkTableEntry D_pidIn = tabele.add("in", 0.0).getEntry();
    private final NetworkTableEntry D_pidOut = tabele.add("Out", 0.0).getEntry();

    //Variables
    private double DirHeader;

    //Subsystem for Elevator
    public Elevator() 
    {
        int i = 3;
        //Elevator motor
        ele_motors = new TitanQuad(Constants.TITAN_ID, i);
        ele_motors.setInverted(true);   //Positive is CW. Need to reverse

        //Limit Switches
        limitSwch = new DigitalInput(10);

        //Move elevator up to contact limit switch and set encoder value to 0
        while (limitSwch.get() == true)
        {
            setElevatorMotorSpeed(0.6);
        }
        setElevatorMotorSpeed(0);
        ele_encoders = new TitanQuadEncoder(ele_motors, i, Constants.eleEncoderDistPerPulse);
        ele_encoders.reset();
        encoderDists = ele_encoders.getEncoderDistance();
        
        //Tune the PID
        pidControllers = new PIDController(3, 0, 0.03, pid_dT);      
    }

    /***
     * 
     * @param z - z speed in m/s
     */
    public void setElevatorSpeed(double z) 
    {
        pidInput = z;
        //ele_motors.set(z);
    }

    public void setElevatorMotorSpeed(double z)
    {
        ele_motors.set(z);
    }

    // public void ResetElevatorPosition() //Must be followed with a 3 second wait command
    // {
    //     while (limitSwch.get() == true)
    //     {
    //         setElevatorMotorSpeed(0.6);
    //     }
    //     setElevatorMotorSpeed(0);
    //     ele_encoders.reset();
    // }

    public void setElevatorPosition(double pos) 
    {
        DirHeader = (-pos - encoderDists) / Math.abs(-pos - encoderDists);
        if (encoderDists >= -pos)
        {
            ele_motors.set(DirHeader * 0.2);
            if (encoderDists <= -pos)
            {
                ele_motors.set(0);
            }
        }
        else
        {
            ele_motors.set(DirHeader * 0.2);
            if (encoderDists >= -pos)
            {
                ele_motors.set(0);
            }
        }    
    }

    public void doEle()
    {
        //Calculates the Encoder Speed and Distance
        encoderSpeeds = -ele_encoders.getSpeed()*Math.PI*Constants.ChainRadius*2/60;
        encoderDists = ele_encoders.getEncoderDistance();
    }

    /**
     * Code that runs once every robot loop
     */

    @Override
    public void periodic()
    {
        //Constantly calls the doEle() function
        doEle();
        //PID Control for the elevator
        pidOutput = pidControllers.calculate(encoderSpeeds, pidInput);
        ele_motors.set(pidOutput);
        /**
         * Updates for outputs to the shuffleboard
         */

        //D_curHeading.setDouble(curHeading);
        //Vmx encoder
        //vmx D_encoderDisp0.setDouble(encoders[0].getDistance());//encoderSpeeds[0]);
        //vmx D_encoderDisp1.setDouble(encoders[1].getDistance());//encoderSpeeds[1]);
        //vmx D_encoderDisp2.setDouble(encoders[2].getDistance());//encoderSpeeds[2]);
        //Titan encoder
        D_encoderDisp3.setDouble(encoderDists);//encoderSpeeds[2]);
        D_inputSwch.setBoolean(limitSwch.get());
        D_pidIn.setDouble(pidInput);
        D_pidOut.setDouble(pidOutput);
  
    }
}