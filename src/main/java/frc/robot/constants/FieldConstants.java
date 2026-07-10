package frc.robot.constants;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.NinjasLib.statemachine.RobotStateBase;

import java.io.IOException;
import java.util.List;
import java.util.Optional;

public class FieldConstants {
    public static AprilTagFieldLayout kBlueFieldLayout;
    public static AprilTagFieldLayout kRedFieldLayout;

    static {
        try {
            kBlueFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2026RebuiltWelded.m_resourceFile);
            kBlueFieldLayout.setOrigin(AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide);

            kRedFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2026RebuiltWelded.m_resourceFile);
            kRedFieldLayout.setOrigin(AprilTagFieldLayout.OriginPosition.kRedAllianceWallRightSide);
        } catch (IOException e) {
            throw new RuntimeException("Unable to load field layout");
        }
    }

    public static Optional<AprilTagFieldLayout> getFieldLayoutWithIgnored(List<Integer> ignoredTags) {
        if (RobotStateBase.getAlliance().isEmpty())
            return Optional.empty();

        AprilTagFieldLayout layout;

        layout = RobotStateBase.getAlliance().get() == DriverStation.Alliance.Blue
            ? kBlueFieldLayout
            : kRedFieldLayout;

        if (!ignoredTags.isEmpty()) {
            List<AprilTag> tags = layout.getTags();
            tags.removeIf(tag -> ignoredTags.contains(tag.ID));
            layout = new AprilTagFieldLayout(tags, layout.getFieldLength(), layout.getFieldWidth());
        }

        return Optional.of(layout);
    }

    public static Optional<AprilTagFieldLayout> getFieldLayoutWithAllowed(List<Integer> allowedTags) {
        if (getFieldLayout().isEmpty())
            return Optional.empty();

        AprilTagFieldLayout layout = getFieldLayout().get();
        if (!allowedTags.isEmpty()) {
            List<AprilTag> tags = layout.getTags();
            tags.removeIf(tag -> !allowedTags.contains(tag.ID));
            layout = new AprilTagFieldLayout(tags, layout.getFieldLength(), layout.getFieldWidth());
        }

        return Optional.of(layout);
    }

    public static Optional<AprilTagFieldLayout> getFieldLayout() {
        return getFieldLayoutWithIgnored(List.of());
    }

    public static Optional<Pose3d> getTagPose(int id) {
        if (getFieldLayout().isEmpty())
            return Optional.empty();

        return Optional.of(getFieldLayout().get().getTagPose(id).get());
    }
}
