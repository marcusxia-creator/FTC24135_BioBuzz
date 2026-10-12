package org.firstinspires.ftc.teamcode.TeleOp.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDController;
import org.firstinspires.ftc.teamcode.TeleOp.UniversalTools.RobotHardware;

public class ShooterSubsystem extends SubsystemBase {
    private final RobotHardware robot;
    private double targetVelocity = 3500.0; // Target ticks per second
    private final double VELOCITY_STEP = 200.0;

    // Custom SolversLib Software PID Controller
    private final PIDController pidController = new PIDController(0.02, 0.0, 0.0);

    public ShooterSubsystem(RobotHardware robot) {
        this.robot = robot;
        if (robot.shooterMotor != null) {
            robot.shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public void runFlywheel() {
        if (robot.shooterMotor != null) {
            double currentVelocity = robot.shooterMotor.getVelocity();
            double pidOutput = pidController.calculate(currentVelocity, targetVelocity);
            pidOutput = Range.clip(pidOutput, 0.0, 1.0);
            robot.shooterMotor.setPower(pidOutput);
        }
    }

    public void increasePower() {
        targetVelocity += VELOCITY_STEP;
    }

    public void decreasePower() {
        targetVelocity = Math.max(0.0, targetVelocity - VELOCITY_STEP);
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public double getCurrentVelocity() {
        return robot.shooterMotor.getVelocity();
    }
    public double getshooterPower() {
        return targetVelocity;
    }

    public void stop() {
        pidController.reset();
        if (robot.shooterMotor != null) {
            robot.shooterMotor.setPower(0);
        }
    }
}
