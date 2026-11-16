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
 * Subsystem for calculating robot heading (tx) relative to the geometric center
 * of a 4-AprilTag cluster using Pure 2D Image Geometry.
 *
 * Does NOT rely on 3D pose or 3D distance estimation, avoiding 3D depth noise & pose jitter!
 */
public class Limelight2DCenterSubsystem extends SubsystemBase {

    private final RobotHardware robot;

    private double rawTx = 0.0;
    private double txToCenter = 0.0;
    private boolean isCenterValid = false;

    // Physical width of a single AprilTag (e.g., 4.0 inches)
    private double physicalTagWidthInches = 4.0;

    // Limelight 3A horizontal Field of View in degrees
    private static final double LIMELIGHT_3A_HORIZONTAL_FOV_DEGREES = 62.8;
    // Camera frame width in pixels (standard Limelight stream resolution)
    private static final double CAMERA_FRAME_WIDTH_PIXELS = 1280.0;

    private final List<Integer> tagDetected = new ArrayList<>();
    private int maxTagCount = 4;

    // Map of Tag ID -> Horizontal X Offset (in inches) from the center of the 4 AprilTags.
    // Negative = Left of center, Positive = Right of center.
    private final Map<Integer, Double> tagXOffsetsInches = new HashMap<>();

    public Limelight2DCenterSubsystem(RobotHardware robot) {
        this.robot = robot;
    }

    /**
     * Configure horizontal X offset (in inches/cm) for a specific AprilTag
     * relative to the center of the 4-tag group.
     * @param tagId AprilTag ID
     * @param xOffsetInches Distance from center (Negative = Left of center, Positive = Right of center)
     */
    public void setTagXOffset(int tagId, double xOffsetInches) {
        tagXOffsetsInches.put(tagId, xOffsetInches);
    }

    /**
     * Configure offsets for all tags in the target group.
     * @param offsetsInches Map of Tag ID to X offset in inches
     */
    public void setTagXOffsets(Map<Integer, Double> offsetsInches) {
        this.tagXOffsetsInches.clear();
        this.tagXOffsetsInches.putAll(offsetsInches);
    }

    /**
     * Set physical width of an individual AprilTag (in inches or cm).
     */
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

            // Filter detected tags that exist in our offset configuration map
            List<ValidTagInfo> validTags = new ArrayList<>();

            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                int tagID = fr.getFiducialId();
                tagDetected.add(tagID);

                Double xOffsetObj = tagXOffsetsInches.get(tagID);
                if (xOffsetObj != null) {
                    double xOffset = xOffsetObj;
                    double tagTx = fr.getTargetXDegrees();
                    double cornerWidthPx = getCornerWidthPixels(fr.getTargetCorners());

                    validTags.add(new ValidTagInfo(tagID, xOffset, tagTx, cornerWidthPx));
                }
            }

            if (validTags.size() >= 2) {
                // =========================================================================
                // PURE 2D METHOD A (2+ Tags Visible): Pairwise Linear Interpolation
                // Formula for pair (i, j): centerTx = (X_j * tx_i - X_i * tx_j) / (X_j - X_i)
                // Zero 3D pose, zero distance calculation needed!
                // =========================================================================
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
                // =========================================================================
                // PURE 2D METHOD B (Only 1 Tag Visible): 2D Corner Pixel Scale Factor
                // Uses visible 2D tag width in pixels to compute local angular scale (deg/inch)
                // =========================================================================
                ValidTagInfo tag = validTags.get(0);

                if (tag.cornerWidthPx > 1.0 && physicalTagWidthInches > 0) {
                    // Angular width of tag in camera degrees
                    double degreesPerPixel = LIMELIGHT_3A_HORIZONTAL_FOV_DEGREES / CAMERA_FRAME_WIDTH_PIXELS;
                    double tagAngularWidthDegrees = tag.cornerWidthPx * degreesPerPixel;

                    // Local scale factor: Degrees per inch at current distance
                    double degreesPerInch = tagAngularWidthDegrees / physicalTagWidthInches;

                    // Shift tag's tx to center: centerTx = tagTx - (xOffset * degreesPerInch)
                    txToCenter = tag.tx - (tag.xOffset * degreesPerInch);
                    isCenterValid = true;
                } else {
                    txToCenter = tag.tx;
                    isCenterValid = true;
                }

            } else if (!fiducialResults.isEmpty()) {
                // Fallback to raw tx if no tags match configured offsets
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
            // Corner 0: top-left, Corner 1: top-right, Corner 2: bottom-right, Corner 3: bottom-left
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
        final double cornerWidthPx;

        ValidTagInfo(int id, double xOffset, double tx, double cornerWidthPx) {
            this.id = id;
            this.xOffset = xOffset;
            this.tx = tx;
            this.cornerWidthPx = cornerWidthPx;
        }
    }

    /**
     * Raw tx returned by Limelight for the primary tag.
     */
    public double getRawTx() {
        return rawTx;
    }

    /**
     * Horizontal angle (tx) in degrees from camera crosshair to target CENTER.
     * Computed using Pure 2D Geometry (no 3D pose noise).
     */
    public double getTxToCenter() {
        return txToCenter;
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
