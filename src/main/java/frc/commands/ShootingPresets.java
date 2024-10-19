package frc.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.constants.TunerConstants;
import frc.robot.RobotContainer;
import frc.subsystems.arm.Arm;
import frc.subsystems.claw.Claw;
import frc.subsystems.flywheel.Flywheel;

public class ShootingPresets {
    // declare shooting-related subsystems
    private Arm arm;
    private Flywheel flywheel;
    private Claw claw;

    public ShootingPresets(Arm arm, Claw claw, Flywheel flywheel) {
        this.arm = arm;
        this.claw = claw;
        this.flywheel = flywheel;
    }

    // Angle and flywheel speeds at each location.
    public static class Settings {
        public static class Subwoofer {
            public static double arm = 0.0;
            public static double leftFlywheel = 6000;
            public static double rightFlywheel = 8000;
        }

        public static class Amp {
            public static double arm = 80.0;
        }

        public static class Field {
            public static double middleCenter = 16;
            public static double middleCorner = 14.7;
            public static double top = 15;
            public static double bottom = 15;
        }

        public static class Shuttle {
            public static double blue = 325.7;
            public static double red = 230.7;
        }
    }

    // Fire next to subwoofer.
    public Command shootClose() {
        return new ParallelCommandGroup(
                new InstantCommand(() -> arm.setGoal(Settings.Subwoofer.arm)),
                flywheel.on(Settings.Subwoofer.leftFlywheel, Settings.Subwoofer.rightFlywheel));
    }

    // Fire next to amp.
    public Command shootAmp() {
        return new InstantCommand(() -> arm.setGoal(Settings.Amp.arm));
    }

    // Fire next to subwoofer and then stop the flywheel during autonomous.
    public Command subwooferAuto() {
        return new SequentialCommandGroup(
                shootClose(),
                new WaitCommand(2),
                // new WaitUntilCommand(() -> flywheel.withinSetGoal()),
                claw.fire(),
                new WaitCommand(0.5),
                flywheel.off());
    }

    public Command shootWhileMoving() {
        return new SequentialCommandGroup(claw.fire(), new WaitCommand(0.5), flywheel.off());
    }

    public Command shootSlap() {
        return new SequentialCommandGroup(claw.fire());
    }

    public Command shootSubNoFly() {
        return new SequentialCommandGroup(claw.fire(), new WaitCommand(0.5));
    }

    public Command shootMiddle() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> arm.setGoal(Settings.Field.middleCenter)),
                new WaitUntilCommand(() -> arm.withinSetGoalTolerance()),
                claw.fire(),
                new WaitCommand(0.5));
    }

    public Command shootMiddleCorner() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> arm.setGoal(Settings.Field.middleCorner)),
                new WaitUntilCommand(() -> arm.withinSetGoalTolerance()),
                claw.fire(),
                new WaitCommand(0.5));
    }

    public Command shootTop() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> arm.setGoal(Settings.Field.top)), claw.fire(), new WaitCommand(0.5));
    }

    public Command shootBottom() {
        return new SequentialCommandGroup(
                // new InstantCommand(() -> arm.setGoal(Settings.field.bottom)),
                claw.fire(), new WaitCommand(0.5));
    }

    public Command shuttle() {
        if (RobotContainer.getInstance().isBlue()) {
            return TunerConstants.DriveTrain.faceAngleWithTolerance(Rotation2d.fromDegrees(Settings.Shuttle.blue));
        }
        return TunerConstants.DriveTrain.faceAngleWithTolerance(Rotation2d.fromDegrees(Settings.Shuttle.red));
    }
}
