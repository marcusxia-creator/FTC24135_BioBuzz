package org.firstinspires.ftc.teamcode.TeleOp.Commands;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.TeleOp.GamepadDriver.GamepadDriver;
import org.firstinspires.ftc.teamcode.TeleOp.GamepadDriver.PrebuiltInputs.DigitalInputs.Button;
import org.firstinspires.ftc.teamcode.TeleOp.GamepadDriver.PrebuiltInputs.DigitalInputs.Toggle;
import org.firstinspires.ftc.teamcode.TeleOp.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.TeleOp.Subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.TeleOp.Subsystems.TransferSubsystem;

public class GamepadControl extends GamepadDriver {

    public Toggle intakeToggle;
    public Button shootButton;

    private final GamepadEx gamepadEx1;
    private final GamepadEx gamepadEx2;

    public GamepadControl(Gamepad gamepad1, Gamepad gamepad2, IntakeSubsystem intake, ShooterSubsystem shooter, TransferSubsystem transfer) {
        double debounce = 0.1;
        Gamepad[] gamepads = new Gamepad[]{gamepad1, gamepad2};

        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);

        intakeToggle = new Toggle(gamepads, g -> g.a, debounce, false);
        shootButton = new Button(gamepads, g -> g.x, debounce);

        // Configure all button bindings right here in GamepadControl
        configureButtonBindings(intake, shooter, transfer);
    }

    private void configureButtonBindings(IntakeSubsystem intake, ShooterSubsystem shooter, TransferSubsystem transfer) {
        // Run intake
        new GamepadButton(gamepadEx1, GamepadKeys.Button.RIGHT_BUMPER)
                .whileHeld(new RunCommand(intake::runRollers, intake))
                .whenReleased(new InstantCommand(intake::stop, intake));

        new GamepadButton(gamepadEx1, GamepadKeys.Button.DPAD_UP)
                .whenPressed(new InstantCommand(intake::increasePower, intake));

        new GamepadButton(gamepadEx1, GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new InstantCommand(intake::decreasePower, intake));

        // Run Shooter
        new GamepadButton(gamepadEx1, GamepadKeys.Button.X)
                .whileHeld(new RunCommand(shooter::runFlywheel, shooter))
                .whenReleased(new InstantCommand(shooter::stop, shooter));

        new GamepadButton(gamepadEx1, GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(new InstantCommand(shooter::increasePower, shooter));

        new GamepadButton(gamepadEx1, GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(new InstantCommand(shooter::decreasePower, shooter));

        // Stop all
        new GamepadButton(gamepadEx1, GamepadKeys.Button.B)
                .whenPressed(new InstantCommand(() -> {
                    shooter.stop();
                    intake.stop();
                }, shooter, intake));
    }
}
