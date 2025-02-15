package frc.robot.lib;

import java.util.Optional;

import org.opencv.core.Mat;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.VisionHelperConstants.RobotPoseConstants;
import frc.robot.subsystems.LLVisionSubsystem;
import edu.wpi.first.math.geometry.Pose3d;


/** Add your docs here. */
public class VisionHelpers {



    public static double getDistanceBetweenCenterCameraCoral(Double alpha, Double beta, double h, double d) {
        if(!alpha.isNaN() && beta.isNaN()){
            return calculateDistanceUsingAlpha(alpha, h, d);
        } else if (alpha.isNaN() && !beta.isNaN()){
            return calculateDistanceUsingAlpha(beta, h, d);
        } else if(!alpha.isNaN() && !beta.isNaN()){
            if(Math.abs(alpha)>beta){
                return calculateDistanceUsingBeta(beta, h, d);
            } else {
                return calculateDistanceUsingAlpha(alpha, h, d);
            }
        }
        return 0;
    }

    public static double calculateDistanceUsingAlpha(Double alpha, double h, double d){
        double x = (h-Math.tan(90 - Math.abs(alpha))*d/2.0)/Math.tan(90 - Math.abs(alpha));
        return x;
    }

    public static double calculateDistanceUsingBeta(Double beta, double h, double d){
        double x = d/2.0 - h*Math.tan(Math.abs(beta));
        return x;
    }

    public static double calculateTheta (double d, Double alpha, Double beta, double h ) {
        double c = (Math.sin(90-beta) * d)/Math.sin(Math.abs(alpha) * Math.abs(beta));
        double x = (Math.sin(90 - Math.abs(alpha)) * c);
        double l = (x / Math.tan(90 - Math.abs(alpha)));
        double y = (-(l * x)/ h) + ((d * x) / (2 * h)); 
        double z = 1 - (x / h);
        double totalLength = y/z;
        return Math.atan(x / totalLength); 
    }

    
        
    /**
     * Get the pose of a specific AprilTag by its ID.
     *
     * @param tagId The ID of the AprilTag.
     * @return The pose of the AprilTag as a Pose3d, or null if the tag is not found.
     */

    

    public static Pose3d getTagPose(int tagId) {
        if (LLVisionSubsystem.fieldLayout != null) {
            Optional<Pose3d> tagPose = LLVisionSubsystem.fieldLayout.getTagPose(tagId);
            if (tagPose.isPresent()) {
                return tagPose.get();
            } else {
                DriverStation.reportWarning("AprilTag " + tagId + " not found in the field layout.", false);
            }
        }
        return null;
    }

    public static void populateRobotPosesHash() {
        RobotPoseConstants.visionRobotPoses.put("Coral Station 1", new Pose2d(33.51, 25.80, new Rotation2d(360 - 54)));
        RobotPoseConstants.visionRobotPoses.put("Coral Station 2", new Pose2d(33.51, 291.20, new Rotation2d(360 - 306)));
        RobotPoseConstants.visionRobotPoses.put("Reef Side 1R", new Pose2d(144.0, 158.50, new Rotation2d(360 - 180)));
        RobotPoseConstants.visionRobotPoses.put("Reef Side 1L", new Pose2d());
        RobotPoseConstants.visionRobotPoses.put("Reef Side 2R", new Pose2d(160.39, 186.83, new Rotation2d(360 - 120)));
        RobotPoseConstants.visionRobotPoses.put("Reef Side 2L", new Pose2d());
        RobotPoseConstants.visionRobotPoses.put("Reef Side 3R", new Pose2d(193.10, 186.83, new Rotation2d(360 - 60)));
        RobotPoseConstants.visionRobotPoses.put("Reef Side 3L", new Pose2d());
        RobotPoseConstants.visionRobotPoses.put("Reef Side 4R", new Pose2d(209.49, 158.50, new Rotation2d(360 - 0)));
        RobotPoseConstants.visionRobotPoses.put("Reef Side 4L", new Pose2d());
        RobotPoseConstants.visionRobotPoses.put("Reef Side 5R", new Pose2d(193.10, 130.17, new Rotation2d(360 - 300)));
        RobotPoseConstants.visionRobotPoses.put("Reef Side 5L", new Pose2d());
        RobotPoseConstants.visionRobotPoses.put("Reef Side 6R", new Pose2d(160.39, 130.17, new Rotation2d(360 - 240)));
        RobotPoseConstants.visionRobotPoses.put("Reef Side 6L", new Pose2d());
        RobotPoseConstants.visionRobotPoses.put("Barge", new Pose2d(325.68, 241.64, new Rotation2d(360 - 180)));
        RobotPoseConstants.visionRobotPoses.put("Processor", new Pose2d(455.15, 317.15, new Rotation2d(360 - 270)));
    }

    /**
     * Check if the AprilTag field layout was loaded successfully.
     *
     * @return True if the field layout is valid, false otherwise.
     */
    public static boolean isFieldLayoutValid() {
        return LLVisionSubsystem.fieldLayout != null;
    }
}
