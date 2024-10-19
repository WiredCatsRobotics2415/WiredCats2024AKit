package frc.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class RobotMeasurements {
    public static final Transform2d IntakeDownTransform = new Transform2d(0.5525, 0, new Rotation2d());

    public static final Transform3d BackLLTransform = new Transform3d(
            new Translation3d(Units.inchesToMeters(-12.815), 0, Units.inchesToMeters(10)),
            new Rotation3d(Units.degreesToRadians(-90), Units.degreesToRadians(30), 0));
}
