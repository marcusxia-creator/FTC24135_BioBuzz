package org.firstinspires.ftc.teamcode.src.Subsystems;

import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.lib.GamepadDriver.GamepadDriver;
import org.firstinspires.ftc.teamcode.src.Commands.ShooterCommand;

/** Keeps all operator button bindings out of the OpMode. */
public class ControlSubsystem extends GamepadDriver {

    public void configureControls(
            GamepadEx gamepad1,
            GamepadEx gamepad2,
            IntakeSubsystem intake,
            ShooterSubsystem shooter) {

        bindIntakeControls(gamepad1, intake);
        bindIntakeControls(gamepad2, intake);

        // X spools the flywheel and automatically feeds once it reaches velocity.
        new GamepadButton(gamepad1, GamepadKeys.Button.X)
                .whenPressed(new ShooterCommand(shooter, intake));
        new GamepadButton(gamepad2, GamepadKeys.Button.X)
                .whenPressed(new ShooterCommand(shooter, intake));

        bindShooterSpeedControls(gamepad2, shooter);
        bindIntakeSpeedControls(gamepad1, intake);
        bindIntakeSpeedControls(gamepad2, intake);

        new GamepadButton(gamepad1, GamepadKeys.Button.B)
                .whenPressed(new InstantCommand(() -> {
                    shooter.stop();
                    intake.stop();
                }, shooter, intake));
        new GamepadButton(gamepad2, GamepadKeys.Button.B)
                .whenPressed(new InstantCommand(() -> {
                    shooter.stop();
                    intake.stop();
                }, shooter, intake));
    }

    private void bindIntakeControls(GamepadEx gamepad, IntakeSubsystem intake) {
        // D-pad Right selects pollen by default; bumper combos select nectar or flower.
        new GamepadButton(gamepad, GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(new InstantCommand(() -> {
                    if (gamepad.getButton(GamepadKeys.Button.RIGHT_BUMPER)) {
                        intake.runRollersNectar();
                    } else if (gamepad.getButton(GamepadKeys.Button.LEFT_BUMPER)) {
                        intake.runRollersFlower();
                    } else {
                        intake.runRollersPollen();
                    }
                }, intake));

        // Explicit chord triggers support either press order. D-pad Left alone always stops.
        Trigger left = new GamepadButton(gamepad, GamepadKeys.Button.DPAD_LEFT);
        Trigger rightBumper = new GamepadButton(gamepad, GamepadKeys.Button.RIGHT_BUMPER);
        left.and(rightBumper.negate())
                .whenActive(new InstantCommand(intake::stop, intake));
        left.and(rightBumper)
                .whenActive(new InstantCommand(intake::reverseIntake, intake));
    }


    private void bindShooterSpeedControls(GamepadEx gamepad, ShooterSubsystem shooter) {
        new GamepadButton(gamepad, GamepadKeys.Button.DPAD_UP)
                .whenPressed(new InstantCommand(() -> {
                    if (!gamepad.getButton(GamepadKeys.Button.DPAD_LEFT)
                            && !gamepad.getButton(GamepadKeys.Button.DPAD_RIGHT)
                            && !gamepad.getButton(GamepadKeys.Button.RIGHT_BUMPER)) {
                        shooter.increaseTargetRPM();
                    }
                }, shooter));
        new GamepadButton(gamepad, GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new InstantCommand(() -> {
                    if (!gamepad.getButton(GamepadKeys.Button.DPAD_LEFT)
                            && !gamepad.getButton(GamepadKeys.Button.DPAD_RIGHT)
                            && !gamepad.getButton(GamepadKeys.Button.LEFT_BUMPER)) {
                        shooter.decreaseTargetRPM();
                    }
                }, shooter));
    }


    private void bindIntakeSpeedControls(GamepadEx gamepad, IntakeSubsystem intake) {
        new GamepadButton(gamepad, GamepadKeys.Button.DPAD_UP)
                .whenPressed(new InstantCommand(() -> {
                    if (gamepad.getButton(GamepadKeys.Button.RIGHT_BUMPER)
                            && !gamepad.getButton(GamepadKeys.Button.DPAD_RIGHT)
                            && !gamepad.getButton(GamepadKeys.Button.DPAD_LEFT)) {
                        intake.increasePower();
                    }
                }, intake));
        new GamepadButton(gamepad, GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new InstantCommand(() -> {
                    if (gamepad.getButton(GamepadKeys.Button.LEFT_BUMPER)
                            && !gamepad.getButton(GamepadKeys.Button.DPAD_RIGHT)
                            && !gamepad.getButton(GamepadKeys.Button.DPAD_LEFT)) {
                        intake.decreasePower();
                    }
                }, intake));
    }

}
