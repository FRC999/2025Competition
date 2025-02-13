package frc.robot.lib;

import java.util.Optional;

import org.opencv.core.Mat;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.LLVisionSubsystem;
import edu.wpi.first.math.geometry.Pose3d;


/** Add your docs here. */
public interface VisionHelpers {

    

    public default double getDistanceBetweenCenterCameraCoral(Double alpha, Double beta, double h, double d) {
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

    public default double calculateDistanceUsingAlpha(Double alpha, double h, double d){
        double x = (h-Math.tan(90 - Math.abs(alpha))*d/2.0)/Math.tan(90 - Math.abs(alpha));
        return x;
    }

    public default double calculateDistanceUsingBeta(Double beta, double h, double d){
        double x = d/2.0 - h*Math.tan(Math.abs(beta));
        return x;
    }

    public default double calculateTheta (double d, Double alpha, Double beta, double h ) {
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

    

    public default Pose3d getTagPose(int tagId) {
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

    /**
     * Check if the AprilTag field layout was loaded successfully.
     *
     * @return True if the field layout is valid, false otherwise.
     */
    public default boolean isFieldLayoutValid() {
        return LLVisionSubsystem.fieldLayout != null;
    }
}
