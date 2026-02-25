package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class Constants {
  public static final int Hood_ID = 30;
  public static final int Flywheel_Lead_ID = 31;
  public static final int Flywheel_Follower_ID = 5;
  public static final int Index_ID = 50;
  public static final int Spindex_ID = 2;

  public static final int kDriverControllerPort = 0;
    public static final int LEFT_LEADER_ID = 1;
    public static double turnVoltage = .3;
    public static boolean stopbutton = false;
    public static double yaw;
    public static double targetYaw = 0.0;
    public static double currentYaw;
    public static double currentSkew;
    public static boolean targetVisible = false;
    public static int framesMissed = 0; //How many frames has the turret not seen the april tag
    public static final double CameraHeight = .635;
    public static final double TagHeight = 1.084;
    public static final double CameraPitch = 0;
    public static final double TagPitch = 0;
// middle of the hub red: 13.0155555556, 4.41236111111
// middle of the hub blue: 5.05861111111, 4.41236111111
    public static double RedHubX = 13.0155555556;
    public static double RedHubY = 4.41236111111;
    public static double BlueHubX = 5.05861111111;
    public static double BlueHubY = 4.41236111111;

    public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    public static final Transform3d kRobotToCam = new Transform3d(new Translation3d(0.0, 0.0, 0.635), new Rotation3d(0, 0, 0));
}
