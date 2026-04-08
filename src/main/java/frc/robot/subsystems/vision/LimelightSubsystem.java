package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
//import edu.wpi.first.wpilibj.DriverStation;
//import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.LimelightHelpers;
import org.littletonrobotics.junction.Logger;

public class LimelightSubsystem extends SubsystemBase {
  private final Drive drive;
  private final String limelightName;

  private Pose2d lastVisionPose = new Pose2d();
  private boolean hasVision = false;
  private boolean target2HUB = false;
  private boolean hasTagCount = false;
  private boolean ignoringScaleError = false;

  private static final double MAX_TAG_DISTANCE_METERS = 4.5;
  private static final double MAX_AMBIGUITY = 0.7; // 0.5 - 0.7
  private static final double MAX_ANGULAR_VEL_DEG_PER_SEC = 300.0; // 180.0

  public LimelightSubsystem(Drive drive, String limelightName) {
    this.drive = drive;
    this.limelightName = limelightName;
  }

  @Override
  public void periodic(){

    boolean connected = isLimelightConnected();
    Logger.recordOutput("Vision/" + limelightName + "/Connected", connected);

    if (!connected) {
      updateDashboard("Disconnected", "None", 0, 0.0, false);
      hasVision = false;
      hasTagCount = false;
      return; 
    }

    /*if (!isLimelightConnected()) {
      Logger.recordOutput("Vision/" + limelightName + "/State", "Disconnected");
      updateDashboard("Disconnected", "None", 0, 0.0, false);
      hasVision = false;
      return;
    }*/

    if (Math.abs(drive.getAngularVelocityDegPerSec()) > MAX_ANGULAR_VEL_DEG_PER_SEC) {
      Logger.recordOutput("Vision/" + limelightName + "/State", "TooFast");
      updateDashboard("TooFast", "None", 0, 0.0, false);
      hasVision = false;
      return;
    }

    /*double yawForLL = drive.getRotation().getDegrees();
    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
        yawForLL += 180.0;
    }*/

    LimelightHelpers.SetRobotOrientation(
        limelightName,
        drive.getRotation().getDegrees(),
        0,
        //drive.getAngularVelocityDegPerSec(),
        0, 0, 0, 0
    );

    LimelightHelpers.PoseEstimate mt2 =
        LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);

    if (mt2 != null && mt2.tagCount >= 1){
      boolean reject = false;

      Matrix<N3, N1> stdDevs = getStdDevsByDistance(mt2.avgTagDist, mt2.pose);
      if (stdDevs == null) reject = true;

      if (!reject) {
        hasTagCount = true;
        drive.addVisionMeasurement(
            mt2.pose,
            mt2.timestampSeconds,
            stdDevs
        );

        lastVisionPose = mt2.pose;
        hasVision = true;

        updateDashboard("OK","MegaTag2",mt2.tagCount,mt2.avgTagDist,true);
        Logger.recordOutput("Vision/" + limelightName + "/Mode", "MegaTag2");
        Logger.recordOutput("Vision/" + limelightName + "/Pose", mt2.pose);
        Logger.recordOutput("Vision/" + limelightName + "/TagCount", mt2.tagCount);
        return;
      }
    }

    LimelightHelpers.PoseEstimate mt1 =
        LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);

    if (mt1 != null  && mt1.tagCount >= 1) {
      boolean reject = false;
      if (mt1.tagCount == 1 && mt1.rawFiducials.length == 1) {
        var tag = mt1.rawFiducials[0];

        if (tag.ambiguity > MAX_AMBIGUITY) reject = true;
        if (tag.distToCamera > MAX_TAG_DISTANCE_METERS) reject = true;
      }

      Matrix<N3, N1> stdDevs = getStdDevsByDistance(mt1.avgTagDist, mt1.pose);
      if (stdDevs == null) reject = true;

      if (!reject) {
        hasTagCount = true;
        drive.addVisionMeasurement(
            mt1.pose,
            mt1.timestampSeconds,
            stdDevs
        );

        lastVisionPose = mt1.pose;
        hasVision = true;

        updateDashboard("OK","MegaTag1",mt1.tagCount,mt1.avgTagDist,true);
        Logger.recordOutput("Vision/" + limelightName + "/Mode", "MegaTag1");
        Logger.recordOutput("Vision/" + limelightName + "/Pose", mt1.pose);
        Logger.recordOutput("Vision/" + limelightName + "/TagCount", mt1.tagCount);
        return;
      }
    }

    updateDashboard("OK", "NoTargets", 0, 0.0, false);
    Logger.recordOutput("Vision/" + limelightName + "/Mode", "NoTargets");
    Logger.recordOutput("Vision/" + limelightName + "/Pose", lastVisionPose);
    Logger.recordOutput("Vision/" + limelightName + "/TagCount", 0);
    hasTagCount = false;
    hasVision = false;
  }

  public boolean getHasTagCount(){
    return hasTagCount;
  }

  public void setEnableAutoaimHUB(boolean enableAutoaimHUB){
    target2HUB = enableAutoaimHUB;
  }

  public void setIgnoringScaleError(boolean disableScaleError){
  ignoringScaleError = disableScaleError;
}

  private Matrix<N3, N1> getStdDevsByDistance(double distanceMeters, Pose2d limePose) {
    //double valRotation = (target2HUB) ? Math.toRadians(1.5) : 9999999;
    double valRotation = (target2HUB) ? 9999999 : 9999999;

    Pose2d estimated = drive.getPoseEstimator().getEstimatedPosition();
    double dx = limePose.getX() - estimated.getX();
    double dy = limePose.getY() - estimated.getY();
    double distError = Math.hypot(dx, dy);
    
    double scale = 1.0;
    if(!ignoringScaleError){
      if(distError <= 0.2){
        scale *= 1.0;        // 1.0
      }else if(distError <= 0.6){
        scale *= 3.0;        // 2.5
      }else if(distError <= 1.0){
        scale *= 5.0;        // 3.5
      }else{          
        scale *= 7.0;       // 4.0
      }
    }

    if(distanceMeters <= 1.5) {
      return VecBuilder.fill(0.5*scale, 0.5*scale, valRotation); // 0.15
    }else if (distanceMeters <= 3.5){
      return VecBuilder.fill(0.7*scale, 0.7*scale, valRotation); // 0.25
    }else if (distanceMeters <= MAX_TAG_DISTANCE_METERS){
      return VecBuilder.fill(1.2*scale, 1.2*scale, valRotation); // 0.6
    }else{
      return null;
    }
  }

  public boolean hasVision() {
    return hasVision;
  }

  /*private boolean isLimelightConnected() {
    return NetworkTableInstance.getDefault()
        .getTable(limelightName)
        .getEntry("tv")
        .getDouble(0) != 0;
  }*/

  private boolean isLimelightConnected() {
      return NetworkTableInstance.getDefault()
          .getTable(limelightName)
          .getEntry("hb")
          .getDouble(0) > 0;
  }

  /*private boolean isLimelightConnected() {
    return NetworkTableInstance.getDefault()
          .getTable(limelightName)
          .getEntry("tv")
          .exists();
  }*/

  private void updateDashboard(
    String state,
    String mode,
    int tagCount,
    double avgDist,
    boolean hasVision){
    SmartDashboard.putString(limelightName + "-State", state);
    SmartDashboard.putString(limelightName + "-Mode", mode);
    SmartDashboard.putNumber(limelightName + "-TagCount", tagCount);
    SmartDashboard.putNumber(limelightName + "-AvgTagDist", avgDist);
    SmartDashboard.putBoolean(limelightName + "-HasVision", hasVision);
  }
}