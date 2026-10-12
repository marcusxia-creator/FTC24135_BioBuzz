package org.firstinspires.ftc.teamcode.TeleOp.Subsystems;

import com.seattlesolvers.solverslib.command.SubsystemBase;
import org.firstinspires.ftc.teamcode.TeleOp.UniversalTools.RobotHardware;

public class TransferSubsystem extends SubsystemBase {
    private final RobotHardware robot;
    private final double SWEEP_POWER = 1.0; // Full speed forward for continuous rotation servo
    private final double STOP_POWER = 0.0;  // Stopped power for CRServo

    public TransferSubsystem(RobotHardware robot) {
        this.robot = robot;
    }

    /**
     * Sweeps the continuous rotation transfer servo to push balls up into the shooter.
     */
    public void sweep() {
        if (robot.transferServo != null) {
            robot.transferServo.setPower(SWEEP_POWER);
        }
    }

    /**
     * Stops the continuous rotation transfer servo.
     */
    public void stop() {
        if (robot.transferServo != null) {
            robot.transferServo.setPower(STOP_POWER);
        }
    }
}
