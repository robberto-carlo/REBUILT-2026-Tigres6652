package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;
import java.util.Queue;

/** IO implementation for QuestNav. */
public class GyroIOQuestNav implements GyroIO {
  private final QuestNav questNav = new QuestNav();
  private final Queue<Double> yawPositionQueue;
  private final Queue<Double> yawTimestampQueue;

  // Previous yaw for velocity calculation
  private double previousYawDegrees = 0.0;
  private double previousTimestamp = 0.0;
  private double yawVelocityRadPerSec = 0.0;

  // Last known yaw to prevent flipping to zero
  private double lastKnownYawDegrees = 0.0;

  // Yaw offset to zero the gyro
  private double yawOffsetDegrees = 0.0;

  public GyroIOQuestNav() {
    yawTimestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
    yawPositionQueue =
        PhoenixOdometryThread.getInstance().registerSignal(this::getCurrentYawDegrees);
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    questNav.commandPeriodic();

    inputs.connected = questNav.isConnected();

    double currentYawDegrees = getCurrentYawDegrees();
    double currentTimestamp = getLatestFrameTimestamp();

    if (previousTimestamp > 0.0 && currentTimestamp > 0.0) {
      double deltaTime = currentTimestamp - previousTimestamp;
      if (deltaTime > 0.0) {
        double deltaYaw = currentYawDegrees - previousYawDegrees;

        while (deltaYaw > 180.0) deltaYaw -= 360.0;
        while (deltaYaw < -180.0) deltaYaw += 360.0;

        yawVelocityRadPerSec = Units.degreesToRadians(deltaYaw / deltaTime);
      }
    }

    previousYawDegrees = currentYawDegrees;
    previousTimestamp = currentTimestamp;

    inputs.yawPosition = Rotation2d.fromDegrees(currentYawDegrees);
    inputs.yawVelocityRadPerSec = yawVelocityRadPerSec;

    inputs.odometryYawTimestamps =
        yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryYawPositions =
        yawPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromDegrees(value))
            .toArray(Rotation2d[]::new);
    yawTimestampQueue.clear();
    yawPositionQueue.clear();
  }

  private double getCurrentYawDegrees() {
    PoseFrame[] poseFrames = questNav.getAllUnreadPoseFrames();
    for (PoseFrame frame : poseFrames) {
      if (!frame.isTracking()) continue;

      Pose3d questPose = frame.questPose3d();
      lastKnownYawDegrees = questPose.getRotation().toRotation2d().getDegrees();
    }

    double offsetYaw = lastKnownYawDegrees - yawOffsetDegrees;

    while (offsetYaw > 180.0) offsetYaw -= 360.0;
    while (offsetYaw < -180.0) offsetYaw += 360.0;

    return offsetYaw;
  }

  private double getLatestFrameTimestamp() {
    PoseFrame[] poseFrames = questNav.getAllUnreadPoseFrames();
    double latest = 0.0;

    for (PoseFrame frame : poseFrames) {
      if (!frame.isTracking()) continue;
      latest = frame.dataTimestamp();
    }

    return latest;
  }

  public void setYawOffset() {
    yawOffsetDegrees = lastKnownYawDegrees;
    System.out.println(
        "Gyro offset set to: " + String.format("%.1f", yawOffsetDegrees) + " degrees");
  }
}
