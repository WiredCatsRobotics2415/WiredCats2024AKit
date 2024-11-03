package frc.constants;

public class Hardware {
    public static final String CanbusName = "rio";

    public static class IntakePorts {
        public static final int IntakeMotorID = 18;
        public static final int FlywheelIR = 5;
    }

    public static class ArmPorts {
        public static final int LeftMotorID = 32;
        public static final int RightMotorID = 14;
        public static final int Potentiometer = 4;
        public static final int LimitSwitch = 1;
    }

    public static class FlywheelPorts {
        public static final int LeftMotorID = 20;
        public static final int RightMotorID = 16;
    }

    public static class ClawPorts {
        public static final int MotorID = 15;
    }

    /** Looking from the BACK of the Motor */
    public static final boolean TalonFXDirectionClockWise = false;
    /** Looking from the BACK of the Motor */
    public static final boolean TalonFXDirectionCounterClockWise = true;

    /** Looking from the BACK of the Motor */
    public static final boolean SparkMaxDirectionClockWise = false;
    /** Looking from the BACK of the Motor */
    public static final boolean SparkMaxDirectionCounterClockWise = true;
}
