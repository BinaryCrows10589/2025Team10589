package frc.robot.Utils.JoystickUtils;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.GenericConstants.ControlConstants;

public class ButtonBoardInterface {
    private GenericHID buttonBoard;
    private Command[][] buttonMap;
    
    public ButtonBoardInterface(int controlerPortID) {
        this.buttonBoard = new GenericHID(controlerPortID);
        this.buttonMap = new Command[this.buttonBoard.getButtonCount()+1][2];
    }

    public void bindButton(Command onTrue, Command onFalse, int buttonIndex) {
        this.buttonMap[buttonIndex][0] = onTrue;
        this.buttonMap[buttonIndex][1] = onFalse; 
    }

    public void bindButton(Command onTrue, int buttonIndex) {
        this.buttonMap[buttonIndex][0] = onTrue;
        this.buttonMap[buttonIndex][1] = null;
    }

    public void periodic() {
        for(int i = 1; i < buttonMap.length; i++) {
            if(this.buttonBoard.getRawButtonPressed(i)) {
                if(this.buttonMap[i][0] != null)
                    this.buttonMap[i][0].schedule();
            }
            if(this.buttonBoard.getRawButtonReleased(i)) {
                if(this.buttonMap[i][1] == null) {
                    if(this.buttonMap[i][0] != null)
                        this.buttonMap[i][0].cancel();
                } else {
                    if(this.buttonMap[i][1] != null)
                        this.buttonMap[i][1].schedule();
                }
            }
        }
    }
    
}