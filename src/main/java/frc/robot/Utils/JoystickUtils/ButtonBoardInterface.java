package frc.robot.Utils.JoystickUtils;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.GenericConstants.ControlConstants;

public class ButtonBoardInterface {
    private GenericHID buttonBoardAutoPositioning; // 12 button auto positioning
    private GenericHID buttonBoardNormal; // 18 (for now) button normals
    private Command[] autoPositioningButtonMap;
    private Command[][] normalButtonMap;
    
    public ButtonBoardInterface(int autoPositioningPortID, int normalPortID) {
        this.buttonBoardAutoPositioning = new GenericHID(autoPositioningPortID);
        this.buttonBoardNormal = new GenericHID(normalPortID);
        this.autoPositioningButtonMap = new Command[this.buttonBoardAutoPositioning.getButtonCount()+1];
        this.normalButtonMap = new Command[this.buttonBoardNormal.getButtonCount()+1][2];
    }
    /*
     * TODO: Find which ports will be for these and list here
     */
    public void bindAutoPositioningCommand(Command autoPositioningCommand, int buttonIndex) {
        this.autoPositioningButtonMap[buttonIndex] = autoPositioningCommand;
    }

    public void bindButton(Command onTrue, Command onFalse, int buttonIndex) {
        this.normalButtonMap[buttonIndex][0] = onTrue;
        this.normalButtonMap[buttonIndex][1] = onFalse; 
    }

    public void bindButton(Command onTrue, int buttonIndex) {
        this.normalButtonMap[buttonIndex][0] = onTrue;
        this.normalButtonMap[buttonIndex][1] = null;
    }

    public BooleanSupplier getNormalButtonSupplier(int buttonIndex) {
        return new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return buttonBoardNormal.getRawButton(buttonIndex);
            }
        };
    }   

    public void periodic() {
        // Normal button board
        for(int i = 1; i < normalButtonMap.length; i++) {
            if(this.buttonBoardNormal.getRawButtonPressed(i)) {
                if(this.normalButtonMap[i][0] != null)
                    this.normalButtonMap[i][0].schedule();
            }
            if(this.buttonBoardNormal.getRawButtonReleased(i)) {
                if(this.normalButtonMap[i][1] == null) {
                    if(this.normalButtonMap[i][0] != null)
                        this.normalButtonMap[i][0].cancel();
                } else {
                    if(this.normalButtonMap[i][1] != null)
                        this.normalButtonMap[i][1].schedule();
                }
            }
        }

        // Auto positioning
        for(int i = 1; i < autoPositioningButtonMap.length; i++) {
            if(this.buttonBoardAutoPositioning.getRawButtonPressed(i)) {
                if(this.autoPositioningButtonMap[i] != null) {
                    this.autoPositioningButtonMap[i].schedule();
                }
            }
            if(this.buttonBoardAutoPositioning.getRawButtonReleased(i)) {
                if(this.autoPositioningButtonMap[i] != null) {
                    this.autoPositioningButtonMap[i].cancel();
                }
            }
        }
    }
    
}