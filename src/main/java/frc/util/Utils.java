package frc.util;

import frc.constants.RuntimeConstants;
import frc.constants.RuntimeConstants.Mode;
import frc.robot.Robot;
import java.lang.reflect.InvocationTargetException;

public class Utils {
    /**
     * Select the correct IO implementation based on the runtime mode.
     *
     * @param real The IO if the robot is on real hardware
     * @param sim The physics simulation IO
     * @param replay The replay IO
     * @return The instantiated IO (MUST be cast to correct IO class)
     */
    public static Object getIOImplementation(Class<?> real, Class<?> sim, Class<?> replay) {
        Class<?> attemptedInstantationType = real;
        try {
            if (Robot.isSimulation()) {
                if (RuntimeConstants.simMode == Mode.REPLAY) {
                    attemptedInstantationType = replay;
                    return replay.getDeclaredConstructors()[0].newInstance();
                } else {
                    attemptedInstantationType = sim;
                    return sim.getDeclaredConstructors()[0].newInstance();
                }
            } else {
                attemptedInstantationType = real;
                return real.getDeclaredConstructors()[0].newInstance();
            }
        } catch (InstantiationException | SecurityException | IllegalAccessException | InvocationTargetException e) {
            System.out.println("WARNING: COULD NOT INSTANTIATE IO IMPLEMENTATION FOR CLASS "
                    + attemptedInstantationType.getName()
                    + ":");
            e.printStackTrace();
        }
        return new Object();
    }
}
