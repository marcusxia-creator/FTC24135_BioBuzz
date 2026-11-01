package org.firstinspires.ftc.teamcode.TeleOp.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import static org.firstinspires.ftc.teamcode.src.UniversalTools.RobotActionConfig.*;
import org.firstinspires.ftc.teamcode.src.UniversalTools.RobotHardware;

@Config
@TeleOp (name = "Test Servos", group = "Test")
public class Test_TeleOp_Servos extends OpMode {
    private RobotHardware robotHardware;
    private GamepadEx gamepad1Ex;
    private GamepadEx gamepad2Ex;
    private ElapsedTime debounceTimer = new ElapsedTime();
    private final double DEBOUNCE_THRESHOLD = 0.15;
    private FtcDashboard dashboard;

    @Override
    public void init() {
        robotHardware = new RobotHardware(hardwareMap);
        robotHardware.init();

        // Initial servo positions from RobotActionConfig
        if (robotHardware.gateServo != null) robotHardware.gateServo.setPosition(gateDownPos);
        if (robotHardware.pollenLimitingServo != null) robotHardware.pollenLimitingServo.setPosition(pollenUpPos);
        if (robotHardware.leftFlowerRampServo != null) robotHardware.leftFlowerRampServo.setPosition(flowerRampUpPos);
        if (robotHardware.rightFlowerRampServo != null) robotHardware.rightFlowerRampServo.setPosition(flowerRampUpPos);
        if (robotHardware.limelightAdjServo != null) robotHardware.limelightAdjServo.setPosition(0.0);

        gamepad1Ex = new GamepadEx(gamepad1);
        gamepad2Ex = new GamepadEx(gamepad2);
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
    }

    private void adjustServo(Servo servo, double delta) {
        if (servo != null) {
            double currentPos = servo.getPosition();
            double newPos = Range.clip(currentPos + delta, 0.0, 1.0);
            servo.setPosition(newPos);
        }
    }

    @Override
    public void loop(){
        // ==================== SERVO CONTROLS (+0.01 / -0.01) ====================

        // 1. Transfer Servo (Gamepad 1: Right/Left Bumper)
        if (gamepad1Ex.getButton(GamepadKeys.Button.RIGHT_BUMPER) && debounceTimer.seconds() >= DEBOUNCE_THRESHOLD) {
            adjustServo(robotHardware.gateServo, 0.01);
            debounceTimer.reset();
        }
        if (gamepad1Ex.getButton(GamepadKeys.Button.LEFT_BUMPER) && debounceTimer.seconds() >= DEBOUNCE_THRESHOLD) {
            adjustServo(robotHardware.gateServo, -0.01);
            debounceTimer.reset();
        }

        // 2. Pollen Limiting Servo (Gamepad 1: Dpad Up/Down)
        if (gamepad1Ex.getButton(GamepadKeys.Button.DPAD_UP) && debounceTimer.seconds() >= DEBOUNCE_THRESHOLD) {
            adjustServo(robotHardware.pollenLimitingServo, 0.01);
            debounceTimer.reset();
        }
        if (gamepad1Ex.getButton(GamepadKeys.Button.DPAD_DOWN) && debounceTimer.seconds() >= DEBOUNCE_THRESHOLD) {
            adjustServo(robotHardware.pollenLimitingServo, -0.01);
            debounceTimer.reset();
        }

        // 3. Flower Ramp Servos - Left & Right Together (Gamepad 2: Dpad Up/Down)
        if (gamepad1Ex.getButton(GamepadKeys.Button.B) && debounceTimer.seconds() >= DEBOUNCE_THRESHOLD) {
            adjustServo(robotHardware.leftFlowerRampServo, 0.01);
            adjustServo(robotHardware.rightFlowerRampServo, 0.01);
            debounceTimer.reset();
        }
        if (gamepad1Ex.getButton(GamepadKeys.Button.X) && debounceTimer.seconds() >= DEBOUNCE_THRESHOLD) {
            adjustServo(robotHardware.leftFlowerRampServo, -0.01);
            adjustServo(robotHardware.rightFlowerRampServo, -0.01);
            debounceTimer.reset();
        }

        // 4. Limelight Adj Servo (Gamepad 2: Y/A Buttons)
        if (gamepad2Ex.getButton(GamepadKeys.Button.Y) && debounceTimer.seconds() >= DEBOUNCE_THRESHOLD) {
            adjustServo(robotHardware.limelightAdjServo, 0.01);
            debounceTimer.reset();
        }
        if (gamepad2Ex.getButton(GamepadKeys.Button.A) && debounceTimer.seconds() >= DEBOUNCE_THRESHOLD) {
            adjustServo(robotHardware.limelightAdjServo, -0.01);
            debounceTimer.reset();
        }

        // ==================== SERVO TELEMETRY ====================
        telemetry.addLine("--- SERVO POSITIONS ---");
        telemetry.addData("1. Gate Servo", "%.2f", robotHardware.gateServo != null ? robotHardware.gateServo.getPosition() : 0.0);
        telemetry.addData("2. Limit Servo", "%.2f", robotHardware.pollenLimitingServo != null ? robotHardware.pollenLimitingServo.getPosition() : 0.0);
        telemetry.addData("3. Left Ramp Servo", "%.2f", robotHardware.leftFlowerRampServo != null ? robotHardware.leftFlowerRampServo.getPosition() : 0.0);
        telemetry.addData("4. Right Ramp Servo", "%.2f", robotHardware.rightFlowerRampServo != null ? robotHardware.rightFlowerRampServo.getPosition() : 0.0);
        telemetry.addData("5. Limelight Servo", "%.2f", robotHardware.limelightAdjServo != null ? robotHardware.limelightAdjServo.getPosition() : 0.0);


        telemetry.update();
    }
}
