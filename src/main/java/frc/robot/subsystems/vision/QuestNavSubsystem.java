package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;
import org.littletonrobotics.junction.Logger;

public class QuestNavSubsystem extends SubsystemBase {
  private final QuestNav questNav;
  private final SwerveDrivePoseEstimator swerveDrivePoseEstimator;

  private static final Transform3d ROBOT_TO_QUEST =
      new Transform3d(
          0.30, 0.33, 0.0, // X, Y, Z en metros
          new Rotation3d(-90.0, 0.0, 0.0)); // Roll, Pitch, Yaw en radianes

  private static final Matrix<N3, N1> BASE_STD_DEVS =
      VecBuilder.fill(0.2, 0.2, Math.toRadians(2));

  public QuestNavSubsystem(SwerveDrivePoseEstimator swerveDrivePoseEstimator) {
    this.swerveDrivePoseEstimator = swerveDrivePoseEstimator;
    questNav = new QuestNav();
  }

  @Override
  public void periodic() {
    questNav.commandPeriodic();

    Logger.recordOutput("QuestNav/Connected", questNav.isConnected());
    Logger.recordOutput("QuestNav/Tracking", questNav.isTracking());
    Logger.recordOutput("QuestNav/Latency", questNav.getLatency());

    try {
      PoseFrame[] poseFrames = questNav.getAllUnreadPoseFrames();
      Logger.recordOutput("QuestNav/PoseFrameCount", poseFrames.length);

      for (PoseFrame frame : poseFrames) {
        if (!frame.isTracking()) continue;

        Pose3d questPose = frame.questPose3d();
        double timestamp = frame.dataTimestamp();

        Pose3d robotPose3d = questPose.transformBy(ROBOT_TO_QUEST.inverse());
        Pose2d robotPose2d = robotPose3d.toPose2d();

        Pose2d estimated = swerveDrivePoseEstimator.getEstimatedPosition();

        double dx = robotPose2d.getX() - estimated.getX();
        double dy = robotPose2d.getY() - estimated.getY();
        double dTheta =
            Math.abs(
                robotPose2d
                    .getRotation()
                    .minus(estimated.getRotation())
                    .getDegrees());

        double distError = Math.hypot(dx, dy);
        double scale = 1.0;
        double scale_degrees = 1.0;


        if (distError > 1.0) {
          scale *= 3.5;
        }else if (distError > 0.5) {
          scale *= 2.5;
        }
        if (dTheta > 30.0) {
          scale_degrees *= 2.0;
        }

        Matrix<N3, N1> adaptiveStdDevs =
            VecBuilder.fill(
                BASE_STD_DEVS.get(0, 0) * scale,
                BASE_STD_DEVS.get(1, 0) * scale,
                BASE_STD_DEVS.get(2, 0) * scale_degrees);

        swerveDrivePoseEstimator.addVisionMeasurement(
            robotPose2d, timestamp, adaptiveStdDevs);

        Logger.recordOutput("QuestNav/Pose", robotPose2d);
        Logger.recordOutput("QuestNav/RawPose3d", questPose);
      }

    } catch (Exception e) {
      Logger.recordOutput("QuestNav/Error", e.getMessage());
    }

    questNav
        .getBatteryPercent()
        .ifPresent(battery -> Logger.recordOutput("QuestNav/BatteryPercent", battery));

    questNav
        .getFrameCount()
        .ifPresent(frameCount -> Logger.recordOutput("QuestNav/FrameCount", frameCount));

    questNav
        .getAppTimestamp()
        .ifPresent(timestamp -> Logger.recordOutput("QuestNav/AppTimestamp", timestamp));

    questNav
        .getTrackingLostCounter()
        .ifPresent(counter -> Logger.recordOutput("QuestNav/TrackingLostCounter", counter));
  }

  public Pose2d getRobotPose() {
    try {
      PoseFrame[] poseFrames = questNav.getAllUnreadPoseFrames();
      if (poseFrames.length > 0) {
        Pose3d questPose = poseFrames[poseFrames.length - 1].questPose3d();
        if (questPose != null) {
          return questPose.transformBy(ROBOT_TO_QUEST.inverse()).toPose2d();
        }
      }
    } catch (Exception e) {
      System.err.println("Error getting QuestNav pose: " + e.getMessage());
    }
    return new Pose2d();
  }

  public void setRobotPose(Pose2d robotPose) {
    Pose3d robotPose3d = new Pose3d(robotPose);
    Pose3d questPose = robotPose3d.transformBy(ROBOT_TO_QUEST);
    questNav.setPose(questPose);
  }

  public boolean isActive() {
    return questNav.isConnected();
  }

  public boolean isTracking() {
    return questNav.isTracking();
  }

  public PoseFrame[] getAllUnreadPoseFrames() {
    return questNav.getAllUnreadPoseFrames();
  }

  public double getLatency() {
    return questNav.getLatency();
  }

  public double getCurrentYawDegrees() {
    return getRobotPose().getRotation().getDegrees();
  }

  public double getCurrentYawRadians() {
    return getRobotPose().getRotation().getRadians();
  }

  public Rotation2d getCurrentRotation() {
    return getRobotPose().getRotation();
  }

  public Translation2d getCurrentPosition() {
    return getRobotPose().getTranslation();
  }
}
