package org.firstinspires.ftc.teamcode.src.Subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.src.UniversalTools.RobotHardware;

import java.util.ArrayList;
import java.util.List;

public class LimelightSubSystem extends SubsystemBase {

    private RobotHardware robot;

    private double tx = 0.0;
    private List<Integer> tagDetected = new ArrayList<>();
    private int maxTagCount = 4;

    public LimelightSubSystem(RobotHardware robot) {
        this.robot = robot;
    }

    private enum Alliance {
        RED,
        BLUE
    }

    private Alliance alliance;

    public void init(Alliance alliance) {
        if (alliance == Alliance.RED) {
            robot.limelight.pipelineSwitch(2);
        }
        if (alliance == Alliance.BLUE) {
            robot.limelight.pipelineSwitch(3);
        }
        robot.limelight.start();
    }

    public void run() {
        LLResult result = robot.limelight.getLatestResult();

        tagDetected.clear();

        if (result != null && result.isValid()) {
            tx = result.getTx();

            List <LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                int tagID = fr.getFiducialId();
                tagDetected.add(tagID);
            }
        }
    }

    public double getTx() {
        return tx;
    }

    public List<Integer> getTagDetected() {
        return tagDetected;
    }

    public boolean isTipped(double percentYield) {
        if (percentYield > 100) {
            percentYield = 100;
        }

        if (percentYield <= 0) {
            percentYield = 25;
        }

        double percentDetected = ((double) tagDetected.size() / maxTagCount) * 100;
        /*
        if (percentDetected > 100) {
            percentDetected = 100;
        }
         */

        return percentDetected >= percentYield;
    }
}
