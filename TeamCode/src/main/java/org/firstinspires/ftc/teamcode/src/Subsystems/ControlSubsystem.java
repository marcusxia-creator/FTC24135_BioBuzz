package org.firstinspires.ftc.teamcode.src.Subsystems;

import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.src.Commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.lib.GamepadDriver.GamepadDriver;
import org.firstinspires.ftc.teamcode.src.Commands.ShooterCommand;

/** Keeps all operator button bindings out of the OpMode. */
public class ControlSubsystem extends GamepadDriver {

    public void configureControls(
            GamepadEx gamepad1,
            GamepadEx gamepad2,
            IntakeSubsystem intake,
            ShooterSubsystem shooter,
            TransferSubsystem transfer) {

        configureGamepad(gamepad1, intake, shooter, transfer, false);
        configureGamepad(gamepad2, intake, shooter, transfer, true);
    }

    private void configureGamepad(
            GamepadEx gamepad,
            IntakeSubsystem intake,
            ShooterSubsystem shooter,
            TransferSubsystem transfer,
            boolean controlsShooterSpeed) {
        bindIntakeControls(gamepad, intake, transfer);
        bindShooterControls(gamepad, intake, shooter, transfer);
        bindIntakeSpeedControls(gamepad, intake);
        if (controlsShooterSpeed) bindShooterSpeedControls(gamepad, shooter);
    }

    private void bindShooterControls(
            GamepadEx gamepad,
            IntakeSubsystem intake,
            ShooterSubsystem shooter,
            TransferSubsystem transfer) {
        new GamepadButton(gamepad, GamepadKeys.Button.X)
                .whenPressed(new ShooterCommand(shooter, intake, transfer));
        new GamepadButton(gamepad, GamepadKeys.Button.B)
                .whenPressed(new InstantCommand(() -> {
                    shooter.stop();
                    intake.stop();
                    transfer.stopTransfer();
                }, shooter, intake, transfer));
    }

    private void bindIntakeControls(
            GamepadEx gamepad,
            IntakeSubsystem intake,
            TransferSubsystem transfer) {
        // Intake and transfer share one command during intake mode.
        new GamepadButton(gamepad, GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(new IntakeCommand(intake, transfer, () -> {
                    if (gamepad.getButton(GamepadKeys.Button.RIGHT_BUMPER)) {
                        return IntakeCommand.IntakeMode.NECTAR;
                    }
                    if (gamepad.getButton(GamepadKeys.Button.LEFT_BUMPER)) {
                        return IntakeCommand.IntakeMode.FLOWER;
                    }
                    return IntakeCommand.IntakeMode.POLLEN;
                }));

        // Explicit chord triggers support either press order. D-pad Left alone always stops.
        Trigger left = new GamepadButton(gamepad, GamepadKeys.Button.DPAD_LEFT);
        Trigger rightBumper = new GamepadButton(gamepad, GamepadKeys.Button.RIGHT_BUMPER);
        left.and(rightBumper.negate())
                .whenActive(new InstantCommand(() -> {
                    intake.stop();
                    transfer.stopTransfer();
                }, intake, transfer));
        left.and(rightBumper)
                .whenActive(new InstantCommand(() -> {
                    intake.reverseIntake();
                    transfer.stopTransfer();
                }, intake, transfer));
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
