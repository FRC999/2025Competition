package frc.robot.lib;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;

public class TrajectoryHelpers {

    public static Pose2d correctedPose = new Pose2d();

    /**
     * This method will adjust the endPose based on the angle.
     * For now it will siply turn it to the specified angle using the startPose as origin.
     * This will likely be sufficient for small trajectory deviations during auto-driving to pickup notes.
     * Note that Rotation2d of the final pose will change.
     * In other words, this routine willl turn endPose by angle using startPose as origin
     */
    public static Pose2d correctEndingPoseBasedOnNoteLocation(Pose2d startPose, Pose2d endPose, double angle) {
        Pose2d relativePoseTurned = endPose.relativeTo(startPose).rotateBy(Rotation2d.fromDegrees(angle));  //NEW POSE BASED ON ANGLE RELATIVE 2 STARTING POSE
        Transform2d t = relativePoseTurned.minus(new Pose2d()); // converts relative pose to a transformation
        return startPose.transformBy(t);
        
    }

    // get corrected X of the pose when turning the endpose using startpose as origin
    // Used to get part of the Transform
    public static double getCorrectedX(Pose2d startPose, Pose2d endPose, double angle) {
        return correctEndingPoseBasedOnNoteLocation(startPose, endPose, angle).getX();
    }

    // get corrected X of the pose when turning the endpose using startpose as origin
    // Used to get part of the Transform
    public static double getCorrectedY(Pose2d startPose, Pose2d endPose, double angle) {
        return correctEndingPoseBasedOnNoteLocation(startPose, endPose, angle).getY();
    }


    public static void setCorrectedPose(Pose2d startPose, Pose2d endPose, double angle) {
        correctedPose = correctEndingPoseBasedOnNoteLocation(startPose, endPose, angle);
    }

    public static Pose2d getCorrectedPose() {
        return correctedPose;
    }

    // How much do I need to rotate from current pose to point to the second pose
    public static Rotation2d rotateToPointToSecondPose(Pose2d currentPose, Pose2d secondPose) {
        return secondPose.relativeTo(currentPose).getTranslation().getAngle();
    }

    public static boolean isValueBetween(double v, double min, double max) {
        return v >= min && v <= max;
    }

    /**
     * Replanning trajectory with different speed and acceleration
     * @param traj
     * @param maxVelocity
     * @param maxAngularVelocity  - rad/s
     * @param maxAcceleration
     * @param maxAngularAcceleration rad/s^2
     * @return
     */
    public static PathPlannerPath replanTrajectory(PathPlannerPath traj, double maxVelocity, 
                double maxAngularVelocity, double maxAcceleration, double maxAngularAcceleration) {
        return new PathPlannerPath(
            traj.getWaypoints()
            , new PathConstraints(
                maxVelocity
                , maxAcceleration
                , maxAngularVelocity
                , maxAngularAcceleration
                )
            , traj.getIdealStartingState()
            , traj.getGoalEndState()
             );
    }

    /**
     * This mehthod will change a trajectory to the one relative to the different starting pose2d.
     * Example - trajectory goes from 1,1,0 to 2,1,0 (meaning forward 1m, no change in heading)
     * Start pose is 4,5,90 (meaning pointing up)
     * So the trajectory will be redone going from 4,5,90 to 4,6,90 (meaning - going 1m forward from robot point of view)
     * @param traj - Trajectory that needs to be modified
     * @param pose - Starting Pose
     * @return
     */
    public static PathPlannerPath changeTrajectoryWithDifferentStartingPose(PathPlannerPath traj, Pose2d pose) {
        
        // Get properties of the current trajectory, so it can be recontrsucted
        List<Pose2d> plist = traj.getPathPoses();
        PathConstraints pc = traj.getGlobalConstraints();
        IdealStartingState istate = traj.getIdealStartingState();
        GoalEndState ges = traj.getGoalEndState();

        List<Pose2d> olist = new ArrayList<>();
        
        olist.add(pose); // First pose in the new trajectory is the starting pose

        Pose2d initialPose = traj.getStartingHolonomicPose().get(); // All subsequent poses need to be transforms in relaton to the initial pose

        int count = 0;
        
        for (Pose2d p : plist) {
            if (count++ > 0) { // The initial pose must be skipped
                /**
                 * p.minus(initialPose) is a difference between current trajectory pose and the initial one
                 * So, we add that difference to the robot pose to get the next transformed pose of the trajectory
                 */
                olist.add
                    (pose.transformBy( 
                        p.minus(initialPose)
                    )
                );
            }
        }

        return new PathPlannerPath(
            PathPlannerPath.waypointsFromPoses(olist ),
            pc,
            new IdealStartingState(
                istate.velocity(),
                istate.rotation().plus(pose.getRotation().minus(initialPose.getRotation()))
            ),
            new GoalEndState(
                ges.velocityMPS(),
                ges.rotation().plus(pose.getRotation().minus(initialPose.getRotation()))
            )
        );

    }

}
