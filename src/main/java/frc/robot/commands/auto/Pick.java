package frc.robot.commands.auto;

import java.util.Map;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Globals;
import frc.robot.RobotContainer;
import frc.robot.commands.auto.PickCommands.ElevatorPick;
import frc.robot.commands.auto.PickCommands.GripperPick;
import frc.robot.commands.auto.PickCommands.RobotPickX;
import frc.robot.subsystems.Sensor;
import frc.robot.subsystems.Vision;

/**
 * DriveMotor class
 * <p>
 * This class creates the inline auto command to drive the motor
 */
public class Pick extends AutoCommand {
    
    private final static Vision m_vision = RobotContainer.m_vision;
    private static double offsetX = -0.05;
    private static double offsetY = 0.15;

    private enum CommandSelector {
        CHIPS, NISSIN, KITKAT, BALL, END;
    }

    static public CommandSelector selectCmd123() {

        /*
        item 0 = chips
             1 = ball
             2 = kitkat
             3 = nissin
        */

        if (Globals.curItem == 0)
            return CommandSelector.CHIPS;
        else if (Globals.curItem == 3)
            return CommandSelector.NISSIN;
        else if (Globals.curItem == 1)
            return CommandSelector.BALL;
         else if (Globals.curItem == 2)
            return CommandSelector.KITKAT;
        else
            return CommandSelector.END;

    }

    public Pick() {

        super(
            // Select one of many commands
            // Selection command in selectCmd123
            /*
             * item 0 = chips 
             * 1 = ball 
             * 2 = kitkat 
             * 3 = nissin
             */
            new SelectCommand(Map.ofEntries(
      
                Map.entry(CommandSelector.CHIPS, 
                    new SequentialCommandGroup(
      
                        new RobotPickX(0, 0, 0, 0.1), 
                        new WaitCommand(0.5), 
                        new ElevatorPick(0.3),
                        new WaitCommand(0.5), 
                        new GripperPick(), 
                        new WaitCommand(0.5)
                        
                    )   
                ),
      
                Map.entry(CommandSelector.NISSIN,
                    new SequentialCommandGroup( 
                        
                        new RobotPickX(3, 0, 0, 0.1),
                        new WaitCommand(0.5), 
                        new ElevatorPick(0.3),
                        new WaitCommand(0.5), 
                        new GripperPick(),
                        new WaitCommand(0.5)
                    )
                ),
      
                Map.entry(CommandSelector.KITKAT, 
                    new SequentialCommandGroup( 
                        
                        new RobotPickX(2, 0, 0, 0.1),
                        new WaitCommand(0.5), 
                        new ElevatorPick(0.3),
                        new WaitCommand(0.5), 
                        new GripperPick(),
                        new WaitCommand(0.5)
                    )
                ),
                            
                Map.entry(CommandSelector.BALL, 
                    new SequentialCommandGroup( 
                        
                        new RobotPickX(1, 0, 0, 0.1),
                        new WaitCommand(0.5), 
                        new ElevatorPick(0.3),
                        new WaitCommand(0.5), 
                        new GripperPick(),
                        new WaitCommand(0.5)
                        )
                ),

                Map.entry(CommandSelector.END, 

                     new InstantCommand(m_vision::getItemBool)
                
                )
            
            ),
                
      
            Pick::selectCmd123
      
            //can use clearGroupedCommands() to reuse commands
            ) 
            
        );

    }



}
