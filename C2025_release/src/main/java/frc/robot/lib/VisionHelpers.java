package frc.robot.lib;

import java.util.Optional;

import org.opencv.core.Mat;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.VisionHelperConstants;
import frc.robot.Constants.SwerveConstants.SwerveChassis;
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

    /**
     * Coral Station sides are LOWER and HIGHER. That means for BLUE the LOWER one is on the RIGHT and HIGHER is on the LEFT
     * For RED it's opposite - LOWER on the LEFT and HIGHER on the RIGHT from driver point of view
     * 
     * Reef sides are:
     * 1 - facing the driver on the corresponding side (tags 18 for BLUE and 7 for RED)
     * 2 - RED - COUNTERCLOCKWISE from it from driver point of view (tags 8 for RED)
     * same for sides 3,4,5,6
     * BLUE - CLOCKWISE from it from driver point of view (tags 19 for BLUE)
     */
    public static void createHashMapOfTags() {
        RobotPoseConstants.visionRobotPoses.put("RedCoralLOW", getTagPose(1).toPose2d());
        RobotPoseConstants.visionRobotPoses.put("RedCoralHIGH", getTagPose(2).toPose2d());
        RobotPoseConstants.visionRobotPoses.put("RedReef1", getTagPose(7).toPose2d());
        RobotPoseConstants.visionRobotPoses.put("RedReef2", getTagPose(8).toPose2d());
        RobotPoseConstants.visionRobotPoses.put("RedReef3", getTagPose(9).toPose2d());
        RobotPoseConstants.visionRobotPoses.put("RedReef4", getTagPose(10).toPose2d());
        RobotPoseConstants.visionRobotPoses.put("RedReef5", getTagPose(11).toPose2d());
        RobotPoseConstants.visionRobotPoses.put("RedReef6", getTagPose(6).toPose2d());
        RobotPoseConstants.visionRobotPoses.put("RedBarge",  getTagPose(5).toPose2d());
        RobotPoseConstants.visionRobotPoses.put("RedProcessor",  getTagPose(3).toPose2d());
        RobotPoseConstants.visionRobotPoses.put("BlueCoralLOW", getTagPose(12).toPose2d());
        RobotPoseConstants.visionRobotPoses.put("BlueCoralHIGH", getTagPose(13).toPose2d());
        RobotPoseConstants.visionRobotPoses.put("BlueReef1", getTagPose(18).toPose2d());
        RobotPoseConstants.visionRobotPoses.put("BlueReef2", getTagPose(19).toPose2d());
        RobotPoseConstants.visionRobotPoses.put("BlueReef3", getTagPose(20).toPose2d());
        RobotPoseConstants.visionRobotPoses.put("BlueReef4", getTagPose(21).toPose2d());
        RobotPoseConstants.visionRobotPoses.put("BlueReef5", getTagPose(22).toPose2d());
        RobotPoseConstants.visionRobotPoses.put("BlueReef6", getTagPose(17).toPose2d());
        RobotPoseConstants.visionRobotPoses.put("BlueBarge",  getTagPose(14).toPose2d());
        RobotPoseConstants.visionRobotPoses.put("BlueProcessor",  getTagPose(16).toPose2d());
    }

    /**
     * Move the pose in a pose-centric (relative) way by X and Y without changing rotation
     * (e.g. if pose points LEFT (Rotation 90 degrees), the move of 0,1 moves the Y of the pose by +1)
     * The Rotation of the pose will not change
     * @param pose
     * @return
     */
    public static Pose2d movePoseXY(Pose2d pose, double x, double y) {
        return pose.transformBy(new Transform2d(x,y,Rotation2d.kZero));
    }

    public static void addRobotPosesForCoralPlacement() {
        movePoseXY (
            RobotPoseConstants.visionRobotPoses.put("RedReef1Left",
                RobotPoseConstants.visionRobotPoses.get("RedReef1")
                    .plus(new Transform2d(0, 0, Rotation2d.k180deg)) // Tag poses look TOWARDS the bot, so need to reverse them 180 degrees for the bot direction placement
                    )
            , -(VisionHelperConstants.bumperWidth + (SwerveChassis.WHEEL_BASE/2.0)) // Coordinates of the center of the bot, so need to move them back half-length of the bot
            , - VisionHelperConstants.distanceBetweenReefPoles/2.0 // Move bot to the left
        );
  
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
