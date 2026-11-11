package org.firstinspires.ftc.teamcode.src.Subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.src.UniversalTools.RobotHardware;

public class Limelight extends SubsystemBase {

    private RobotHardware robot;

    private double tx = 0.0;
    private int[] tagDetected;

    public Limelight(RobotHardware robot) {
        this.robot = robot;
    }

    public void init() {
        robot.limelight.start();
    }

    public void init(int setPipeline) {
        robot.limelight.pipelineSwitch(setPipeline);
        robot.limelight.start();
    }

    public void run() {
        LLResult result = robot.limelight.getLatestResult();

        if (result != null && result.isValid()) {
            tx = result.getTx();
            //tagDetected = []
        }
    }

    public double getTx() {
        return tx;
    }

    public boolean isTipped() {
        return false;
    }
}
