package org.firstinspires.ftc.teamcode.src.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;

import org.firstinspires.ftc.teamcode.lib.CommandBase.Action;
import org.firstinspires.ftc.teamcode.src.UniversalTools.RobotHardware;
import static org.firstinspires.ftc.teamcode.src.UniversalTools.RobotActionConfig.*;

@Config
public class ShooterSubsystem extends SubsystemBase {
    private ElapsedTime shootingTime = new ElapsedTime();
    private boolean isShooting = false;
    private final RobotHardware robot;

    public static double leftShooterKp = 0.001, leftShooterKi = 0.0, leftShooterKd = 0.0, leftShooterKf = 0.001;
    public static double rightShooterKp = 0.001, rightShooterKi = 0.0, rightShooterKd = 0.0, rightShooterKf = 0.001;

    public static double leftShooterTargetVelocity = 1100.0;
    public static double rightShooterTargetVelocity = 1100.0;

    private final PIDFController leftShooterPIDF = new PIDFController(leftShooterKp, leftShooterKi, leftShooterKd, leftShooterKf);
    private final PIDFController rightShooterPIDF = new PIDFController(rightShooterKp, rightShooterKi, rightShooterKd, rightShooterKf);

    public ShooterSubsystem(RobotHardware robot) {
        this.robot = robot;
        robot.leftShooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightShooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void runFlywheel() {
        robot.pollenLimitingServo.setPosition(pollenDownPos);
        robot.gateServo.setPosition(gateUpPos);

        leftShooterPIDF.setPIDF(leftShooterKp, leftShooterKi, leftShooterKd, leftShooterKf);
        rightShooterPIDF.setPIDF(rightShooterKp, rightShooterKi, rightShooterKd, rightShooterKf);

        double leftVel = robot.leftShooterMotor.getVelocity();
        double leftOutput = leftShooterPIDF.calculate(leftVel, leftShooterTargetVelocity);
        robot.leftShooterMotor.setPower(Range.clip(leftOutput, 0.0, 1.0));

        double rightVel = robot.rightShooterMotor.getVelocity();
        double rightOutput = rightShooterPIDF.calculate(rightVel, rightShooterTargetVelocity);
        robot.rightShooterMotor.setPower(Range.clip(rightOutput, 0.0, 1.0));
    }

    /**
     * Separate method for feeding/shooting with a 0.5-second timer delay before opening the pollen gate.
     */
    public void shootFlywheel() {
        // First frame of shooting: reset the timer and hold gate down
        if (!isShooting) {
            isShooting = true;
            shootingTime.reset();
            robot.pollenLimitingServo.setPosition(pollenDownPos);
        }
        robot.gateServo.setPosition(gateUpPos);

        robot.intakeMotor.setPower(0.9);

        // After 0.5s, raise gate to pollenUpPos; otherwise keep it at pollenDownPos
        if (shootingTime.seconds() >= 0.75) {
            robot.pollenLimitingServo.setPosition(pollenUpPos);
        } else {
            robot.pollenLimitingServo.setPosition(pollenDownPos);
        }
    }

    public void increasePower() {
        leftShooterTargetVelocity += 200.0;
        rightShooterTargetVelocity += 200.0;
    }

    public void decreasePower() {
        leftShooterTargetVelocity = Math.max(0.0, leftShooterTargetVelocity - 200.0);
        rightShooterTargetVelocity = Math.max(0.0, rightShooterTargetVelocity - 200.0);
    }

    public double getShooterVelocity() {
        return leftShooterTargetVelocity;
    }

    public double getTargetVelocity() {
        return leftShooterTargetVelocity;
    }

    public void stop() {
        // Reset shooting state so next call starts fresh
        isShooting = false;
        shootingTime.reset();

        leftShooterPIDF.reset();
        rightShooterPIDF.reset();
        robot.leftShooterMotor.setPower(0);
        robot.rightShooterMotor.setPower(0);
        robot.intakeMotor.setPower(0);
        robot.pollenLimitingServo.setPosition(pollenDownPos);
    }

    public class spoolFlywheelAction implements Action {
        @Override
        public void loop() {
            runFlywheel();//Constantly run flywheel pid continuously without shooting
        }

        @Override
        public boolean finished() {
            return true;
        }
    }

    public class ShootAction implements Action {
        @Override
        public void loop() {
            shootFlywheel();//retracts limiter for shooting
        }

        @Override
        public void shutdown() {
            stop();
        }

        @Override
        public boolean finished() {
            return true;
        }
    }
}
