package frc.constants;

public final class RuntimeConstants {
    /** What to do if in simulation */
    public static final Mode simMode = Mode.SIM;

    public static enum Mode {
        /** Running a physics simulator. */
        SIM,

        /** Replaying from a log file. */
        REPLAY
    }
}
