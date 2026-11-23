package org.firstinspires.ftc.teamcode.src.Subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.src.UniversalTools.RobotHardware;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Deque;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.IntToDoubleFunction;

/**
 * Subsystem for calculating robot heading (tx) relative to the geometric center
 * of a 4-AprilTag cluster using Pure 2D Image Geometry.
 *
 * Does NOT rely on 3D pose or 3D distance estimation, avoiding 3D depth noise & pose jitter!
 */
public class LimelightSubsystem extends SubsystemBase {

    private final Deque<Double> historyMeasurements = new ArrayDeque<>();
    private final int historySize = 3;
    private final RobotHardware robot;
    private static final long MAX_AIM_FRAME_AGE_MS = 150;
    // FTC SDK BIOBUZZ cluster offsets, ordered by tag ID.
    private static final double[] CELL_TAG_X_OFFSETS_INCHES = {-6.5, -2.75, 2.75, 6.5};
    private LLResult latestResult;
    private double lastFrameTimestamp = Double.NaN;
    private long lastFrameAdvanceNanos;

    private double rawTx = 0.0;
    private double txToCenter = 0.0;
    private boolean isCenterValid = false;

    // Physical width of a single AprilTag (e.g., 4.0 inches)
    private double physicalTagWidthInches = 3.25;

    // Limelight 3A horizontal Field of View in degrees
    private static final double LIMELIGHT_3A_HORIZONTAL_FOV_DEGREES = 54.5;
    // Camera frame width in pixels (standard Limelight stream resolution)
    private static final double CAMERA_FRAME_WIDTH_PIXELS = 640;

    private final List<Integer> tagDetected = new ArrayList<>();
    private int maxTagCount = 4;

    // Map of Tag ID -> Horizontal X Offset (in inches) from the center of the 4 AprilTags.
    // Negative = Left of center, Positive = Right of center.
    private final Map<Integer, Double> tagXOffsetsInches = new HashMap<>();

    public LimelightSubsystem(RobotHardware robot) {
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
        if (robot.limelight != null) {
            robot.limelight.pipelineSwitch(setPipeline);
            robot.limelight.start();
        }
    }

    public void init() {
        if (robot.limelight != null) {
            robot.limelight.start();
        }
    }

    public void stop() {
        if (robot.limelight != null) {
            robot.limelight.stop();
        }
        latestResult = null;
    }

    public void run() {
        LLResult result = robot.limelight == null ? null : robot.limelight.getLatestResult();
        latestResult = result;
        if (result != null && Double.isFinite(result.getTimestamp())
                && result.getTimestamp() != lastFrameTimestamp) {
            lastFrameTimestamp = result.getTimestamp();
            lastFrameAdvanceNanos = System.nanoTime();
        }
        tagDetected.clear();

        if (result != null && result.isValid()) {
            rawTx = result.getTx();
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();

            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                tagDetected.add(fr.getFiducialId());
            }

            List<ValidTagInfo> validTags = collectTags(fiducialResults,
                    id -> tagXOffsetsInches.getOrDefault(id, Double.NaN));
            double center = calculateCenterTx(validTags);
            isCenterValid = !fiducialResults.isEmpty();
            txToCenter = Double.isFinite(center) ? center
                    : !validTags.isEmpty() ? validTags.get(0).tx
                    : isCenterValid ? rawTx : 0.0;
        } else {
            txToCenter = 0.0;
            isCenterValid = false;
        }
    }

    /** Returns the selected cell's center angle, or NaN when its tags cannot provide one. */
    public double getAimTxDegrees(int firstTagId) {
        LLResult result = latestResult;
        if (result == null || !result.isValid() || !Double.isFinite(result.getTimestamp())) {
            return Double.NaN;
        }
        long age = result.getStaleness();
        if (age < 0 || age > MAX_AIM_FRAME_AGE_MS
                || System.nanoTime() - lastFrameAdvanceNanos > MAX_AIM_FRAME_AGE_MS * 1_000_000L) {
            return Double.NaN;
        }

        List<ValidTagInfo> tags = collectTags(result.getFiducialResults(), id -> {
            int index = id - firstTagId;
            return index >= 0 && index < CELL_TAG_X_OFFSETS_INCHES.length
                    ? CELL_TAG_X_OFFSETS_INCHES[index] : Double.NaN;
        });
        return calculateCenterTx(tags);
    }

    private List<ValidTagInfo> collectTags(List<LLResultTypes.FiducialResult> results,
                                          IntToDoubleFunction offsetForTag) {
        List<ValidTagInfo> tags = new ArrayList<>();
        for (LLResultTypes.FiducialResult result : results) {
            int id = result.getFiducialId();
            double offset = offsetForTag.applyAsDouble(id);
            double tx = result.getTargetXDegrees();
            if (Double.isFinite(offset) && Double.isFinite(tx)) {
                tags.add(new ValidTagInfo(id, offset, tx, getCornerWidthPixels(result.getTargetCorners())));
            }
        }
        return tags;
    }

    private double calculateCenterTx(List<ValidTagInfo> tags) {
        double sumCenterTx = 0.0;
        int pairCount = 0;
        for (int i = 0; i < tags.size(); i++) {
            for (int j = i + 1; j < tags.size(); j++) {
                ValidTagInfo first = tags.get(i);
                ValidTagInfo second = tags.get(j);
                double deltaX = second.xOffset - first.xOffset;
                if (Math.abs(deltaX) > 0.001) {
                    // Evaluate the line through the two tag angles at the cell center (x = 0).
                    sumCenterTx += (second.xOffset * first.tx - first.xOffset * second.tx) / deltaX;
                    pairCount++;
                }
            }
        }
        if (pairCount > 0) {
            return sumCenterTx / pairCount;
        }
        if (tags.isEmpty()) {
            return Double.NaN;
        }
        ValidTagInfo tag = tags.get(0);
        if (tag.xOffset == 0.0) {
            return tag.tx;
        }
        if (tag.cornerWidthPx <= 1.0 || !(physicalTagWidthInches > 0.0)) {
            return Double.NaN;
        }
        double degreesPerPixel = LIMELIGHT_3A_HORIZONTAL_FOV_DEGREES / CAMERA_FRAME_WIDTH_PIXELS;
        double degreesPerInch = tag.cornerWidthPx * degreesPerPixel / physicalTagWidthInches;
        return tag.tx - tag.xOffset * degreesPerInch;
    }

    @Override
    public void periodic() {
        run();
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
