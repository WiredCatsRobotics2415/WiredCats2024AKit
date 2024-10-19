package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.constants.DriverControl;
import java.util.HashMap;
import java.util.Map;

public class OIs {
    public enum Bindings {
        PigeonReset,
        LowerArm,
        RaiseArm,
        FixArm,
        Amp,
        ArmDrivePreset,
        ArmIntakePosition,
        ArmAngle,
        Intake,
        AutoIntake,
        ManualOuttake,
        ManualIntake,
        SpinOff,
        SpinUpToAmp,
        Shoot,
        ReverseClaw,
        ShootClose,
        FixAll
    }

    public abstract static class OI {
        static SendableChooser<Integer> oiChooser;

        static {
            oiChooser = new SendableChooser<Integer>();
            oiChooser.setDefaultOption("Gulikit Controller", 0);
            SmartDashboard.putData("OI", oiChooser);
        }

        /** A convenience record to store two doubles without making a whole class */
        public static record TwoDControllerInput(double x, double y) {}
        ;

        /** The binds map of an OI */
        public Map<Bindings, Trigger> binds = new HashMap<Bindings, Trigger>();

        // JOYSTICKS
        /** Get appropriately scaled translation values, in raw controller units [-1, 1] */
        public abstract TwoDControllerInput getXY();

        /** Get appropriately scaled rotation values, in raw controller units [-1, 1] */
        public abstract double getRotation();

        // UTILS

        public abstract XboxController getHIDOfController();
    }

    public static class GulikitController extends OI {
        CommandXboxController controller;
        CommandJoystick numpad;

        private boolean isCurve = DriverControl.UseCurve;
        private double curve = DriverControl.CurveExponent;
        private double slewRate = DriverControl.SlewRate;
        private double deadband = DriverControl.Deadband;

        private SlewRateLimiter xLimiter = new SlewRateLimiter(slewRate);
        private SlewRateLimiter yLimiter = new SlewRateLimiter(slewRate);

        public GulikitController() {
            controller = new CommandXboxController(0);
            numpad = new CommandJoystick(1);

            binds.put(Bindings.PigeonReset, controller.button(7, Robot.buttonEventLoop)); // Minus
            // binds.put("TargetHotspot", controller.button(8, Robot.buttonEventLoop)); //Plus (in use by
            // claw reverse)

            // Arm
            binds.put(Bindings.LowerArm, controller.button(5, Robot.buttonEventLoop)); // left bumper
            binds.put(Bindings.RaiseArm, controller.button(6, Robot.buttonEventLoop)); // right bumper

            // Intake
            binds.put(Bindings.Intake, controller.button(2, Robot.buttonEventLoop)); // A
            binds.put(Bindings.ManualOuttake, controller.leftTrigger());
            binds.put(Bindings.ManualIntake, numpad.button(8, Robot.buttonEventLoop));

            // Flywheel (testing, using controller)
            // binds.put("SpinUp", controller.button(3, Robot.buttonEventLoop)); // Y
            // binds.put("SpinOff", controller.button(4, Robot.buttonEventLoop)); // X
            binds.put(Bindings.SpinOff, numpad.button(8, Robot.buttonEventLoop));
            binds.put(Bindings.SpinUpToAmp, numpad.button(9, Robot.buttonEventLoop));

            binds.put(Bindings.Shoot, controller.axisLessThan(3, 0.5)); // right trigger, 1 at rest and 0 when pressed
            binds.put(Bindings.ReverseClaw, controller.button(8, Robot.buttonEventLoop));

            // binds.put("ShuttleRotate", numpad.button(1, Robot.buttonEventLoop)); //Numpad 1

            // binds.put("ArmDistanceCalculated", controller.button(3, Robot.buttonEventLoop));

            // Presets
            binds.put(Bindings.Amp, numpad.button(4, Robot.buttonEventLoop));
            binds.put(Bindings.ArmDrivePreset, numpad.button(5, Robot.buttonEventLoop));
            binds.put(Bindings.ArmIntakePosition, numpad.button(6, Robot.buttonEventLoop));
            binds.put(Bindings.ShootClose, numpad.button(7, Robot.buttonEventLoop)); // Subwoofer

            binds.put(Bindings.AutoIntake, controller.button(1, Robot.buttonEventLoop)); // B
            binds.put(Bindings.FixAll, numpad.button(1, Robot.buttonEventLoop));
            binds.put(Bindings.ArmAngle, numpad.button(2, Robot.buttonEventLoop));

            binds.put(Bindings.FixArm, numpad.button(3, Robot.buttonEventLoop));
        }

        private double deadbandCompensation(double r) {
            return (r - deadband) / (1 - deadband);
        }

        private double minimumPowerCompensation(double r) {
            return r * (1 - DriverControl.MinimumDrivePower) + DriverControl.MinimumDrivePower;
        }

        public TwoDControllerInput getXY() {
            double x = MathUtil.applyDeadband(controller.getRawAxis(1), deadband);
            double y = MathUtil.applyDeadband(controller.getRawAxis(0), deadband);
            double newX, newY = 0.0d;
            if (isCurve) {
                double angle = Math.atan2(y, x);
                double magInitial = Math.sqrt(x * x + y * y);
                double magCurved = Math.pow(deadbandCompensation(magInitial), curve);
                double powerCompensated = minimumPowerCompensation(magCurved);
                newX = Math.cos(angle) * powerCompensated;
                newY = Math.sin(angle) * powerCompensated;
            } else {
                newX = xLimiter.calculate(x);
                newY = yLimiter.calculate(y);
            }
            if (Double.isNaN(newX)) newX = 0.0d;
            if (Double.isNaN(newY)) newY = 0.0d;
            return new TwoDControllerInput(newX, newY);
        }

        public double getRotation() {
            double deadbandCompensated =
                    deadbandCompensation(MathUtil.applyDeadband(controller.getRawAxis(4), deadband));
            if (isCurve) {
                return Math.pow(minimumPowerCompensation(deadbandCompensated), curve);
            } else {
                return minimumPowerCompensation(deadbandCompensated);
            }
        }

        public XboxController getHIDOfController() {
            return controller.getHID();
        }
    }
}
