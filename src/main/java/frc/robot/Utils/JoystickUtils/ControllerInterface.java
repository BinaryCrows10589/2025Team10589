package frc.robot.Utils.JoystickUtils;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ControlConstants;

public class ControllerInterface {
    
    private XboxController controller;

    /**
     * Creates a Controller Interface. Easier interface to works with and it also works with our FightBox Button Board
     * @param controllerPortID Integer: The USB Port ID of the controller.(Can be found in driver station program)
     */
    public ControllerInterface(int controllerPortID) {
        this.controller = new XboxController(controllerPortID);
    }

    /**
     * Binds a button to a command. One triggers when pressed and the other trigures when the button is let go of
     * @param onTrue Command: The command that runs when the button is pressed
     * @param onFalse Command: The command that runs when that button is let go of
     * @param buttonIndex Integer: The ID of the button. You can use XboxController.Button.* ware stare is the button you want bind the commands to. e.g A, X ect.
     */
    public void bindToButton(Command onTrue, Command onFalse, int buttonIndex) {
        final JoystickButton boundButton = new JoystickButton(this.controller, buttonIndex);
        if(onTrue != null) {
            boundButton.onTrue(onTrue);
        }
        if(onFalse != null) {
            boundButton.onFalse(onFalse);
        }
    }

    /**
     * Binds a button to a command.
     * @param command Command: The command that runs when the button is pressed and is canceled when the button is let go of
     * @param buttonIndex Integer: The ID of the button. You can use XboxController.Button.* ware stare is the button you want bind the commands to. e.g A, X ect.
     */
    public void bindToButton(Command command, int buttonIndex) {
        final JoystickButton boundButton = new JoystickButton(this.controller, buttonIndex);
        if(command != null) {
            boundButton.onTrue(command);
            boundButton.onFalse(Commands.runOnce(()-> command.cancel()));
        }
    }

    /**
     * Binds a button to a command. One triggers when pressed and the other trigures when the button is let go of
     * @param onTrue Command: The command that runs when the button is pressed
     * @param onFalse Command: The command that runs when that button is let go of
     * @param POV Integer: The angle of the POV. 0 is up, 90 is right, 180 is down, 270 is left.  
     */
    public void bindToPOV(Command onTrue, Command onFalse, int POV) {
        final POVButton joystickButton = new POVButton(this.controller, POV);
        if(onTrue != null) {
            joystickButton.onTrue(onTrue);
        }
        if(onFalse != null) {
            joystickButton.onFalse(onFalse);
        }
    }

    /**
     * Binds a button to a command. One triggers when pressed and the other trigures when the button is let go of
     * @param command Command: The command that runs when the button is pressed and is concelted when it is let go of
     * @param POV Integer: The angle of the POV. 0 is up, 90 is right, 180 is down, 270 is left.  
     */
    public void bindToPOV(Command command, int POV) {
        final POVButton joystickButton = new POVButton(this.controller, POV);
        if(command != null) {
            joystickButton.onTrue(command);
            joystickButton.onFalse(Commands.runOnce(()-> command.cancel()));
        }
    }
    
    /**
     * Binds a triggure to a command. One triggers when pressed and the other triggures when the button is let go of
     * @param onTrue Command: The command that runs when the triggure is pressed
     * @param onFalse Command: The command that runs when that triggure is let go of
     */
    public void bindToLeftTriggure(Command onTrue, Command onFalse) {
        BooleanSupplier triggureBooleanSupplier = () -> {return this.controller.getLeftTriggerAxis() >= .2;};
        Trigger trigger = new Trigger(triggureBooleanSupplier);

        if(onTrue != null) {
            trigger.onTrue(onTrue);
        }
        if(onFalse != null) {
            trigger.onFalse(onFalse);
        }
    }

    /**
     * Binds a triggure to a command. One triggers when pressed and the other triggures when the button is let go of
     * @param command Command: The command that runs when the triggure is pressed and is canseled when it is let go of
     */
    public void bindToLeftTriggure(Command command) {
        BooleanSupplier triggureBooleanSupplier = () -> {return this.controller.getLeftTriggerAxis() >= .2;};
        Trigger trigger = new Trigger(triggureBooleanSupplier);

        if(command != null) {
            trigger.onTrue(command);
            trigger.onFalse(Commands.runOnce(()-> command.cancel()));
        }

    }

     /**
     * Binds a triggure to a command. One triggers when pressed and the other triggures when the button is let go of
     * @param command Command: The command that runs when the triggure is pressed and is canseled when it is let go of
     */
    public void bindToRightTriggure(Command command) {
        BooleanSupplier triggureBooleanSupplier = () -> {return this.controller.getRightTriggerAxis() >= .2;};
        Trigger trigger = new Trigger(triggureBooleanSupplier);

        if(command != null) {
            trigger.onTrue(command);
            trigger.onFalse(Commands.runOnce(()-> command.cancel()));
        }
    }

    /**
     * Returns the Left Joysticks X axis value after the deadband has bean applied.
     * @return Double: The Left Joysticks x axis value after the deadband has bean applied.
     */
    public double getLeftX() {
        return modifyAxis(this.controller.getLeftX());
    }

    /**
     * Returns the Right Joysticks X axis value after the deadband has bean applied.
     * @return Double: The Right Joysticks x axis value after the deadband has bean applied.
     */
    public double getRightX() {
        return modifyAxis(this.controller.getRightX());
    }

    /**
     * Returns the Left Joysticks Y axis value after the deadband has bean applied.
     * @return Double: The Left Joysticks Y axis value after the deadband has bean applied.
     */
    public double getLeftY() {
        return modifyAxis(this.controller.getLeftY());
    }

    /**
     * Returns the Right Joysticks Y axis value after the deadband has bean applied.
     * @return Double: The Right Joysticks Y axis value after the deadband has bean applied.
     */
    public double getRightY() {
        return modifyAxis(this.controller.getRightY());
    }

    /**
     * Returns the Left Joysticks X axis value before deadband has bean applied.
     * @return Double: The Left Joysticks x axis value before the deadband has bean applied.
     */
    public double getLeftXRaw() {
        return this.controller.getLeftX();
    }

    /**
     * Returns the Right Joysticks X axis value before the deadband has bean applied.
     * @return Double: The Right Joysticks x axis value before the deadband has bean applied.
     */
    public double getRightXRaw() {
        return this.controller.getRightX();
    }

    /**
     * Returns the Left Joysticks Y axis value before the deadband has bean applied.
     * @return Double: The Left Joysticks Y axis value before the deadband has bean applied.
     */
    public double getLeftYRaw() {
        return this.controller.getLeftY();
    }

    /**
     * Returns the Right Joysticks Y axis value before the deadband has bean applied.
     * @return Double: The Right Joysticks Y axis value before the deadband has bean applied.
     */
    public double getRightYRaw() {
        return this.controller.getRightY();
    }

    /**
     * Returns The Left Bumper's value
     * @return Boolean: The Left Bumper's value
     */
    public boolean getLeftBumperValue() {
        return this.controller.getLeftBumper();
    }

    /**
     * Returns The Right Bumper's value
     * @return Boolean: The Right Bumper's value
     */
    public boolean getRightBumperValue() {
        return this.controller.getRightBumper();
    }

    /**
     * Returns The Left Triggure's value
     * @return Boolean: The Left Triggure's value
     */
    public boolean getLeftTriggureValue() {
        return this.controller.getLeftTriggerAxis() != 0;
    }

     /**
     * Returns The Right Triggure's value
     * @return Boolean: The Right Triggure's value
     */
    public boolean getRightTriggureValue() {
        return this.controller.getRightTriggerAxis() != 0;
    }
    

    /**
     * Modifes the value return by appling a deadband to it. The deadband can be set in the ControllerConstant Files
     * @param value
     * @return Double: The joystick value with the deadband applied. 
     */
    private static double modifyAxis(double value) {
        // Deadband
        value = MathUtil.applyDeadband(value, ControlConstants.kDeadband);
        // Square the axis
        value = Math.copySign(value * value, value);
        return value;
    }

    /**
     * Gets the actual controller object
     * @return XboxController: The actual XboxController object
     */
    public XboxController getController() {
        return controller;
    }

}
