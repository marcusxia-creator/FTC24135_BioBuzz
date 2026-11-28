package org.firstinspires.ftc.teamcode.src.Subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.src.UniversalTools.RobotHardware;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * Subsystem for automatically identifying and calculating robot heading (tx) and distance
 * relative to the geometric center of 4 preset AprilTag clusters using Pure 2D Image Geometry.
 *
 * Automatic Cluster Selection (No Odometry):
 * Selects active cluster based on Tag Count -> Average 'ty' Height (tilted UP side) -> Target Area.
 */
public class LimelightAutoCenter extends SubsystemBase {

    private final RobotHardware robot;

    private double rawTx = 0.0;
    private double txToCenter = 0.0;
    private double distanceToCenterInches = 0.0;
    private boolean isCenterValid = false;
    private TagCluster activeCluster = null;

    // Physical width of a single AprilTag (in inches)
    private double physicalTagWidthInches = 3.25;

    // Limelight 3A horizontal Field of View in degrees & resolution
    private static final double LIMELIGHT_3A_HORIZONTAL_FOV_DEGREES = 54.5;
    private static final double CAMERA_FRAME_WIDTH_PIXELS = 640.0;

    // Camera mounting & field target geometry (for single-tag ty vertical trig)
    private double cameraHeightInches = 12.0;  // Height of camera lens center from floor
    private double targetHeightInches = 65.6;  // Height of AprilTag target center from floor
    private double cameraPitchDegrees = 0.0;   // Tilt angle (+ Up, - Down)

    // Detected tag tracking
    private final List<Integer> tagDetected = new ArrayList<>();
    private int maxTagCount = 4;

    // =========================================================================
    // APRILTAG CLUSTERS
    // =========================================================================
    public enum TagCluster {
        CLUSTER_1(Map.of(
            38, -6.5,
            39, -2.75,
            40, 2.75,
            41, 6.5
        )),
        CLUSTER_2(Map.of(
            34, -6.5,
            35, -2.75,
            36, 2.75,
            37, 6.5
        )),
        CLUSTER_3(Map.of(
            45, -6.5,
            44, -2.75,
            43, 2.75,
            42, 6.5
        )),
        CLUSTER_4(Map.of(
            33, -6.5,
            32, -2.75,
            31, 2.75,
            30, 6.5
        ));

        public final Map<Integer, Double> tagXOffsets;

        TagCluster(Map<Integer, Double> tagXOffsets) {
            this.tagXOffsets = tagXOffsets;
        }
    }

    public LimelightAutoCenter(RobotHardware robot) {
        this.robot = robot;
    }

    /**
     * Configure camera mounting geometry for single-tag ty vertical trigonometry.
     */
    public void setCameraMountingParameters(double cameraHeightInches, double cameraPitchDegrees) {
        this.cameraHeightInches = cameraHeightInches;
        this.cameraPitchDegrees = cameraPitchDegrees;
    }

    public void setTargetHeightInches(double targetHeightInches) {
        this.targetHeightInches = targetHeightInches;
    }

    public void setPhysicalTagWidthInches(double widthInches) {
        this.physicalTagWidthInches = widthInches;
    }

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

            // -----------------------------------------------------------------
            // STEP 1: Group all detected tags by their respective TagCluster
            // -----------------------------------------------------------------
            Map<TagCluster, List<ValidTagInfo>> clusterMap = new HashMap<>();

            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                int tagID = fr.getFiducialId();
                tagDetected.add(tagID);

                // Match tag ID to preset clusters
                for (TagCluster cluster : TagCluster.values()) {
                    Double xOffset = cluster.tagXOffsets.get(tagID);
                    if (xOffset != null) {
                        double tagTx = fr.getTargetXDegrees();
                        double tagTy = fr.getTargetYDegrees();
                        double area = fr.getTargetArea();
                        double cornerWidthPx = getCornerWidthPixels(fr.getTargetCorners());

                        clusterMap.computeIfAbsent(cluster, k -> new ArrayList<>())
                                  .add(new ValidTagInfo(tagID, xOffset, tagTx, tagTy, area, cornerWidthPx));
                    }
                }
            }

            // -----------------------------------------------------------------
            // FIND BEST CLUSTER
            // -----------------------------------------------------------------
            TagCluster bestCluster = null;
            int maxCount = 0;
            double highestAvgTy = -999.0;
            double maxAvgArea = -1.0;

            for (Map.Entry<TagCluster, List<ValidTagInfo>> entry : clusterMap.entrySet()) {
                List<ValidTagInfo> tagsInCluster = entry.getValue();
                int count = tagsInCluster.size();

                double sumTy = 0.0;
                double sumArea = 0.0;
                for (ValidTagInfo tag : tagsInCluster) {
                    sumTy += tag.ty;
                    sumArea += tag.area;
                }
                double avgTy = sumTy / count;
                double avgArea = sumArea / count;

                // Decision Rule
                if (count > maxCount || 
                   (count == maxCount && avgTy > highestAvgTy) || 
                   (count == maxCount && Math.abs(avgTy - highestAvgTy) < 0.1 && avgArea > maxAvgArea)) {
                    
                    maxCount = count;
                    highestAvgTy = avgTy;
                    maxAvgArea = avgArea;
                    bestCluster = entry.getKey();
                }
            }

            activeCluster = bestCluster;

            // -----------------------------------------------------------------
            // STEP 3: Compute center tx and distance for selected active cluster
            // -----------------------------------------------------------------
            if (bestCluster != null && clusterMap.containsKey(bestCluster)) {
                List<ValidTagInfo> validTags = clusterMap.get(bestCluster);

                if (validTags != null && validTags.size() >= 2) {
                    // =========================================================
                    // MULTI-TAG MODE (2+ Tags): Pairwise 2D Linear Interpolation
                    // =========================================================
                    double sumCenterTx = 0.0;
                    int pairCount = 0;

                    for (int i = 0; i < validTags.size(); i++) {
                        for (int j = i + 1; j < validTags.size(); j++) {
                            ValidTagInfo t1 = validTags.get(i);
                            ValidTagInfo t2 = validTags.get(j);

                            double deltaX = t2.xOffset - t1.xOffset;
                            if (Math.abs(deltaX) > 0.001) {
                                double centerTxPair = (t2.xOffset * t1.tx - t1.xOffset * t2.tx) / deltaX;
                                sumCenterTx += centerTxPair;
                                pairCount++;
                            }
                        }
                    }

                    if (pairCount > 0) {
                        txToCenter = sumCenterTx / pairCount;
                        isCenterValid = true;
                    } else {
                        txToCenter = validTags.get(0).tx;
                        isCenterValid = true;
                    }

                } else if (validTags.size() == 1) {
                    // =========================================================
                    // SINGLE-TAG MODE (1 Tag): Vertical 'ty' Angle Trigonometry
                    // =========================================================
                    ValidTagInfo tag = validTags.get(0);

                    double angleRad = Math.toRadians(cameraPitchDegrees + tag.ty);
                    double tanAngle = Math.tan(angleRad);

                    if (-0.1 > tanAngle || tanAngle > 0.1) { /// Valid non-zero vertical angle
                        double distanceInches = (targetHeightInches - cameraHeightInches) / tanAngle;
                        distanceToCenterInches = Math.abs(distanceInches);

                        double angularOffsetDegrees = Math.toDegrees(Math.atan2(tag.xOffset, distanceToCenterInches));
                        txToCenter = tag.tx - angularOffsetDegrees;
                    } else {
                        // Fallback when angle approaches 0
                        txToCenter = 0.0;
                    }
                    isCenterValid = true;

                } else {
                    txToCenter = 0.0;
                    isCenterValid = false;
                }
            } else if (!fiducialResults.isEmpty()) {
                // Fallback to raw tx if no tags match configured clusters
                txToCenter = rawTx;
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
     * Calculates average width of the detected AprilTag in camera pixels from its 4 corners.
     */
    private double getCornerWidthPixels(List<List<Double>> corners) {
        if (corners == null || corners.size() < 4) return 0.0;
        try {
            double topWidth = Math.hypot(corners.get(1).get(0) - corners.get(0).get(0), corners.get(1).get(1) - corners.get(0).get(1));
            double botWidth = Math.hypot(corners.get(2).get(0) - corners.get(3).get(0), corners.get(2).get(1) - corners.get(3).get(1));
            return (topWidth + botWidth) / 2.0;
        } catch (Exception e) {
            return 0.0;
        }
    }

    private static class ValidTagInfo {
        final int id;
        final double xOffset;
        final double tx;
        final double ty;
        final double area;
        final double cornerWidthPx;

        ValidTagInfo(int id, double xOffset, double tx, double ty, double area, double cornerWidthPx) {
            this.id = id;
            this.xOffset = xOffset;
            this.tx = tx;
            this.ty = ty;
            this.area = area;
            this.cornerWidthPx = cornerWidthPx;
        }
    }

    /**
     * Returns raw tx for primary tag.
     */
    public double getRawTx() {
        return rawTx;
    }

    /**
     * Horizontal angle (tx) in degrees from camera crosshair to active cluster CENTER.
     */
    public double getTxToCenter() {
        return txToCenter;
    }

    /**
     * Estimated horizontal distance in inches to target wall / active cluster center.
     */
    public double getDistanceToCenterInches() {
        return distanceToCenterInches;
    }

    /**
     * Returns the currently active facing cluster selected by vision.
     */
    public TagCluster getActiveCluster() {
        return activeCluster;
    }

    public boolean isCenterValid() {
        return isCenterValid;
    }

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
