package org.firstinspires.ftc.teamcode.TeleOp.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.TeleOp.UniversalTools.RobotHardware;

@Config
@TeleOp (name = "Test Dual PID Motors", group = "Test")
public class Test_TeleOp extends OpMode {
    private RobotHardware robotHardware;
    private GamepadEx gamepad1Ex;
    private ElapsedTime debounceTimer = new ElapsedTime();
    private final double DEBOUNCE_THRESHOLD = 0.15;
    private FtcDashboard dashboard;

    public static double leftShooterKp = 0.001, leftShooterKi = 0, leftShooterKd = 0;
    public static double rightShooterKp = 0.001, rightShooterKi = 0, rightShooterKd = 0;
    public static double intakeKp = 0.001, intakeKi = 0, intakeKd = 0;

    private PIDController leftShooterPID = new PIDController(leftShooterKp, leftShooterKi, leftShooterKd);
    private PIDController rightShooterPID = new PIDController(rightShooterKp, rightShooterKi, rightShooterKd);
    private PIDController intakePID = new PIDController(intakeKp, intakeKi, intakeKd);

    public static double leftShooterTargetVelocity = 0.0;
    public static double rightShooterTargetVelocity = 0.0;
    public static double intakeTargetVelocity = 0.0;

    @Override
    public void init() {
        robotHardware = new RobotHardware(hardwareMap);
        robotHardware.init();

        gamepad1Ex = new GamepadEx(gamepad1);
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
    }

    @Override
    public void loop(){
        leftShooterPID.setPID(leftShooterKp, leftShooterKi, leftShooterKd);
        rightShooterPID.setPID(rightShooterKp, rightShooterKi, rightShooterKd);
        intakePID.setPID(intakeKp, intakeKi, intakeKd);

        // --- Shooter Controls ---
        if (gamepad1Ex.getButton(GamepadKeys.Button.X) && debounceTimer.seconds() >= DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            leftShooterTargetVelocity = 2000.0;
            rightShooterTargetVelocity = 2000.0;
        }

        if (gamepad1Ex.getButton(GamepadKeys.Button.Y) && debounceTimer.seconds() >= DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            leftShooterTargetVelocity = 0.0;
            rightShooterTargetVelocity = 0.0;
        }

        if (gamepad1Ex.getButton(GamepadKeys.Button.DPAD_UP) && debounceTimer.seconds() >= DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            leftShooterTargetVelocity += 100.0;
        }

        if (gamepad1Ex.getButton(GamepadKeys.Button.DPAD_DOWN) && debounceTimer.seconds() >= DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            leftShooterTargetVelocity -= 100.0;
        }

        if (gamepad1Ex.getButton(GamepadKeys.Button.DPAD_RIGHT) && debounceTimer.seconds() >= DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            rightShooterTargetVelocity += 100.0;
        }

        if (gamepad1Ex.getButton(GamepadKeys.Button.DPAD_LEFT) && debounceTimer.seconds() >= DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            rightShooterTargetVelocity -= 100.0;
        }

        // --- Intake Controls ---
        if (gamepad1Ex.getButton(GamepadKeys.Button.A) && debounceTimer.seconds() >= DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            intakeTargetVelocity = 650.0;
        }
        if (gamepad1Ex.getButton(GamepadKeys.Button.B) && debounceTimer.seconds() >= DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            intakeTargetVelocity = 0.0;
        }

        // ==================== MOTOR PID CALCULATIONS ====================

        if (robotHardware.leftShooterMotor != null) {
            double leftShooterCurrentVelocity = robotHardware.leftShooterMotor.getVelocity();
            double leftShooterPidOutput = leftShooterPID.calculate(leftShooterCurrentVelocity, leftShooterTargetVelocity);
            robotHardware.leftShooterMotor.setPower(Range.clip(leftShooterPidOutput, 0, 1));
            telemetry.addData("Left Shooter Current", leftShooterCurrentVelocity);
            telemetry.addData("Left Shooter Power", "%.2f", robotHardware.leftShooterMotor.getPower());
        }

        if (robotHardware.rightShooterMotor != null) {
            double rightShooterCurrentVelocity = robotHardware.rightShooterMotor.getVelocity();
            double rightShooterPidOutput = rightShooterPID.calculate(rightShooterCurrentVelocity, rightShooterTargetVelocity);
            robotHardware.rightShooterMotor.setPower(Range.clip(rightShooterPidOutput, 0, 1));
            telemetry.addData("Right Shooter Current", rightShooterCurrentVelocity);
            telemetry.addData("Right Shooter Power", "%.2f", robotHardware.rightShooterMotor.getPower());
        }

        if (robotHardware.intakeMotor != null) {
            double intakeCurrentVelocity = robotHardware.intakeMotor.getVelocity();
            double intakePidOutput = intakePID.calculate(intakeCurrentVelocity, intakeTargetVelocity);
            robotHardware.intakeMotor.setPower(Range.clip(intakePidOutput, 0, 1));
            telemetry.addData("Intake Current", intakeCurrentVelocity);
            telemetry.addData("Intake Power", "%.2f", robotHardware.intakeMotor.getPower());
        }

        telemetry.update();
    }
}
