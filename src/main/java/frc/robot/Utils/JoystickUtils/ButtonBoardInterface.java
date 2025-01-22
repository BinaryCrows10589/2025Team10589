package frc.robot.Utils.JoystickUtils;

import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.Constants.GenericConstants.ControlConstants;

public class ButtonBoardInterface {
    private GenericHID buttonBoard = new GenericHID(ControlConstants.kButtonBoardPort); 

 

    public void periodic() {
        
    }
}