package frc.robot.Utils.JoystickUtils;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.GenericConstants.ControlConstants;

public class ButtonBoardInterface {
    private GenericHID buttonBoard;
    private Command[][] buttonMap = new Command[this.buttonBoard.getButtonCount()][2];
    
    public ButtonBoardInterface(int controlerPortID) {
        this.buttonBoard = new GenericHID(controlerPortID);
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
        for(int i = 0; i < buttonMap.length; i++) {
            int buttonIndex = i+1;
            if(this.buttonBoard.getRawButtonPressed(buttonIndex)) {
                this.buttonMap[buttonIndex][0].schedule();
            }
            if(this.buttonBoard.getRawButtonReleased(buttonIndex)) {
                if(this.buttonMap[buttonIndex][1] == null) {
                    this.buttonMap[buttonIndex][0].cancel();
                } else {
                    this.buttonMap[buttonIndex][1].schedule();
                }
            }
        }
    }
    
}