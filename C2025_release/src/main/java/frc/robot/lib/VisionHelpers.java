package frc.robot.lib;

import org.opencv.core.Mat;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

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
}