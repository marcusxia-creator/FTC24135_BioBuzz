package org.firstinspires.ftc.teamcode.src.Subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.src.UniversalTools.RobotHardware;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * Subsystem for calculating the robot heading (tx) relative to the geometric center
 * of a cluster of 4 AprilTags, even when only 1, 2, 3, or 4 tags are detected.
 */
public class LimelightCenterSubsystem extends SubsystemBase {

    private final RobotHardware robot;

    private double rawTx = 0.0;
    private double txToCenter = 0.0;
    private double distanceToCenterInches = 0.0;
    private boolean isCenterValid = false;

    private final List<Integer> tagDetected = new ArrayList<>();
    private int maxTagCount = 4;
    // Map of Tag ID -> Horizontal X Offset (in inches) from the center of the 4 AprilTags.
    // Negative values = Left of center, Positive values = Right of center.
    private final Map<Integer, Double> tagXOffsetsInches = new HashMap<>();

    public LimelightCenterSubsystem(RobotHardware robot) {
        this.robot = robot;
    }

    /**
     * Configure the relative horizontal X offset (in inches or cm) for a specific AprilTag
     * relative to the center of the 4-tag group.
     * @param tagId AprilTag ID
     * @param xOffsetInches Distance from center (Negative = Left of center, Positive = Right of center)
     */
    public void setTagXOffset(int tagId, double xOffsetInches) {
        tagXOffsetsInches.put(tagId, xOffsetInches);
    }

    /**
     * Configure offsets for all tags in the 4-AprilTag target group.
     * @param offsetsInches Map of Tag ID to X offset in inches
     */
    public void setTagXOffsets(Map<Integer, Double> offsetsInches) {
        this.tagXOffsetsInches.clear();
        this.tagXOffsetsInches.putAll(offsetsInches);
    }

    /**
     * Set expected maximum tag count (default is 4).
     */
    public void setMaxTagCount(int maxTagCount) {
        this.maxTagCount = maxTagCount;
    }

    public void init(int setPipeline) {
        robot.limelight.pipelineSwitch(setPipeline);
        robot.limelight.start();
    }

    public void init() {
        robot.limelight.start();
    }

    public void run() {
        LLResult result = robot.limelight.getLatestResult();

        tagDetected.clear();

        if (result != null && result.isValid()) {
            rawTx = result.getTx();

            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            double sumTxCenter = 0.0;
            double sumDistance = 0.0;
            int validCount = 0;

            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                int tagID = fr.getFiducialId();
                tagDetected.add(tagID);

                // 1. Get distance to this tag using 3D target space pose if available
                Pose3D robotPoseTargetSpace = fr.getRobotPoseTargetSpace();
                double distanceInches = 0.0;

                if (robotPoseTargetSpace != null && robotPoseTargetSpace.getPosition() != null) {
                    double x = robotPoseTargetSpace.getPosition().x;
                    double z = robotPoseTargetSpace.getPosition().z;
                    distanceInches = Math.hypot(x, z);
                }

                // Fallback distance estimation if 3D pose distance is unavailable or near zero
                if (distanceInches <= 0.01) {
                    double area = fr.getTargetArea();
                    if (area > 0) {
                        distanceInches = 100.0 / Math.sqrt(area); // Rough proportional estimate
                    } else {
                        distanceInches = 36.0; // Default estimate (3 feet)
                    }
                }

                double tagTx = fr.getTargetXDegrees();

                // 2. If tag offset is configured in our map, calculate center prediction
                Double xOffsetObj = tagXOffsetsInches.get(tagID);
                if (xOffsetObj != null) {
                    double xOffset = xOffsetObj;

                    // Angular offset of tag relative to center: atan2(xOffset, distance)
                    double angularOffsetDegrees = Math.toDegrees(Math.atan2(xOffset, distanceInches));

                    // Center tx predicted by this tag: tagTx - angularOffset
                    double predictedCenterTx = tagTx - angularOffsetDegrees;

                    sumTxCenter += predictedCenterTx;
                    sumDistance += distanceInches;
                    validCount++;
                }
            }

            if (validCount > 0) {
                txToCenter = sumTxCenter / validCount;
                distanceToCenterInches = sumDistance / validCount;
                isCenterValid = true;
            } else if (!fiducialResults.isEmpty()) {
                // Fallback to primary tag's raw tx if no tags match configured offsets
                txToCenter = rawTx;
                distanceToCenterInches = 0.0;
                isCenterValid = true;
            } else {
                txToCenter = 0.0;
                isCenterValid = false;
            }
        } else {
            txToCenter = 0.0;
            isCenterValid = false;
        }
    }

    /**
     * Gets raw tx returned by Limelight (primary tag offset).
     */
    public double getRawTx() {
        return rawTx;
    }

    /**
     * Gets the horizontal angle (tx) in degrees from the camera crosshair
     * to the CENTER point of the 4 AprilTags.
     * Works when 1, 2, 3, or 4 tags are detected!
     */
    public double getTxToCenter() {
        return txToCenter;
    }

    /**
     * Gets estimated distance in inches to the center of the 4 AprilTags.
     */
    public double getDistanceToCenterInches() {
        return distanceToCenterInches;
    }

    /**
     * Returns true if at least one tag was detected and center tx was computed.
     */
    public boolean isCenterValid() {
        return isCenterValid;
    }

    /**
     * Gets list of all detected AprilTag IDs.
     */
    public List<Integer> getTagDetected() {
        return tagDetected;
    }

    public boolean isTipped(double percentYield) {
        if (percentYield > 100) percentYield = 100;
        if (percentYield <= 0) percentYield = 25;

        double percentDetected = ((double) tagDetected.size() / maxTagCount) * 100;
        return percentDetected >= percentYield;
    }
}
