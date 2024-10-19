package frc.constants;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.commands.Hotspot;
import java.util.ArrayList;

public class FieldMeasurements {
    private static final Translation3d BlueSpeaker = new Translation3d(0.225, 5.55, 2.1);
    private static final Translation3d RedSpeaker = new Translation3d(16.317, 5.55, 2.1);

    public static final Translation2d getSpeakerLocation() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red
                    ? RedSpeaker.toTranslation2d()
                    : BlueSpeaker.toTranslation2d();
        }
        return BlueSpeaker.toTranslation2d();
    }

    public static final Pose3d getSpeakerLocationAsPose() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red
                    ? new Pose3d(RedSpeaker, new Rotation3d())
                    : new Pose3d(BlueSpeaker, new Rotation3d());
        }
        return new Pose3d(BlueSpeaker, new Rotation3d());
    }

    public static final ArrayList<Hotspot> Hotspots = new ArrayList<Hotspot>();

    static {
        Hotspots.add(new Hotspot(136.5, 200));
        Hotspots.add(new Hotspot(155, 236.5));
        Hotspots.add(new Hotspot(136.5, 243.5));
        Hotspots.add(new Hotspot(241.5, 295));
        Hotspots.add(new Hotspot(241.5, 238));
        Hotspots.add(new Hotspot(241.5, 181));
        Hotspots.add(new Hotspot(194.5, 112.638));
        Hotspots.add(new Hotspot(194.5, 324));
    }

    public static final ArrayList<Translation3d> DefaultFieldNotes = new ArrayList<Translation3d>();

    static {
        // Blue alliance side
        DefaultFieldNotes.add(new Translation3d(Units.inchesToMeters(114), Units.inchesToMeters(161.638), 0));
        DefaultFieldNotes.add(new Translation3d(Units.inchesToMeters(114), Units.inchesToMeters(218.638), 0));
        DefaultFieldNotes.add(new Translation3d(Units.inchesToMeters(114), Units.inchesToMeters(275.638), 0));
        // Centerline
        DefaultFieldNotes.add(new Translation3d(Units.inchesToMeters(325.6115), Units.inchesToMeters(29.638), 0));
        DefaultFieldNotes.add(new Translation3d(Units.inchesToMeters(325.6115), Units.inchesToMeters(95.638), 0));
        DefaultFieldNotes.add(new Translation3d(Units.inchesToMeters(325.6115), Units.inchesToMeters(161.638), 0));
        DefaultFieldNotes.add(new Translation3d(Units.inchesToMeters(325.6115), Units.inchesToMeters(227.638), 0));
        DefaultFieldNotes.add(new Translation3d(Units.inchesToMeters(325.6115), Units.inchesToMeters(293.638), 0));
    }

    public static final double NoteDiameter = Units.inchesToMeters(14);
}
