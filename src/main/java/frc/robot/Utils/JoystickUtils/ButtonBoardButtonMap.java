package frc.robot.Utils.JoystickUtils;

public enum ButtonBoardButtonMap {
        /** Left bumper. */
        kBottomRightBlack(5),
        /** Right bumper. */
        kTopRightBlack(6),
        /** Left stick. */
        kWhiteLeft(9),
        /** Right stick. */
        kWhiteRight(10),
        /** A. */
        kGreen(1),
        /** B. */
        kRed(2),
        /** X. */
        kBlue(3),
        /** Y. */
        kYellow(4),
        /** Back. */
        kBlackSelect(7),
        /** Start. */
        kBlackStart(8),
        kTopLeftBlack(11),
        kBottomLeftBlack(12);
    
        /** Button value. */
        public final int value;
        public boolean previousValue;
    
        private ButtonBoardButtonMap(int value) {
          this.value = value;
        }
    }
