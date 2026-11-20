package org.firstinspires.ftc.teamcode.src.Subsystems;

import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.src.Commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.src.Commands.ShooterCommand;
import org.firstinspires.ftc.teamcode.src.Commands.EjectIntakeCommand;

import org.firstinspires.ftc.teamcode.src.UniversalTools.DistanceShooterRPM;
import org.firstinspires.ftc.teamcode.src.UniversalTools.ShotTargeting;


/** Keeps all operator button bindings out of the OpMode. */
public class ControlSubsystem {

    /** Registers the button bindings for both gamepads during TeleOp setup. */
    public void configureControls(
            GamepadEx gamepad1,
            GamepadEx gamepad2,
            IntakeSubsystem intake,
            TransferSubsystem transfer,
            ShooterSubsystem shooter,
            DistanceShooterRPM rpmSelector,
            ShotTargeting targeting) {

        // Select the tipped-up cell without interrupting driving or shooting.
        new GamepadButton(gamepad2, GamepadKeys.Button.LEFT_STICK_BUTTON)
                .whenPressed(new InstantCommand(() -> {
                    targeting.toggleCell();
                    rpmSelector.clearShot();
                }));
        
        //binding keys intake control shooter control transfer control
        bindIntakeControls(gamepad1, intake, transfer);
        bindIntakeControls(gamepad2, intake, transfer);
        bindShooterControls(gamepad1, gamepad2, intake, transfer, shooter, rpmSelector);

        // Gamepad 2 can trim shooter RPM. Both gamepads can adjust intake speed with a bumper.
        bindShooterRPMControls(gamepad2, rpmSelector);
        bindIntakeSpeedControls(gamepad1, intake);
        bindIntakeSpeedControls(gamepad2, intake);
        bindTransferSpeedControls(gamepad1, transfer);
        bindTransferSpeedControls(gamepad2, transfer);
        bindStopControls(gamepad1, intake, transfer, shooter, rpmSelector);
        bindStopControls(gamepad2, intake, transfer, shooter, rpmSelector);
    }

    //methods for controls - key bindings
    private void bindShooterControls(
            GamepadEx gamepad1,
            GamepadEx gamepad2,
            IntakeSubsystem intake,
            TransferSubsystem transfer,
            ShooterSubsystem shooter,
            DistanceShooterRPM rpmSelector) {
        // X starts shooting. On gamepad 2, hold Right Trigger and press X to toggle RPM mode Auto to Manual.
        // Read the trigger on the X press so releasing it requires a fresh X press to shoot.
        ShooterCommand shoot = new ShooterCommand(shooter, intake, transfer, rpmSelector);
        new GamepadButton(gamepad1, GamepadKeys.Button.X)
                .whenPressed(shoot);
        new GamepadButton(gamepad2, GamepadKeys.Button.X)
                .whenPressed(new InstantCommand(() -> {
                    if (gamepad2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5) {
                        rpmSelector.toggleMode();
                    } else {
                        shoot.schedule();
                    }
                }));
    }

    private void bindStopControls(
            GamepadEx gamepad,
            IntakeSubsystem intake,
            TransferSubsystem transfer,
            ShooterSubsystem shooter,
            DistanceShooterRPM rpmSelector) {
        // B stops the shooter, intake, and transfer and clears the selected shot.
        new GamepadButton(gamepad, GamepadKeys.Button.B)
                .whenPressed(new InstantCommand(() -> {
                    shooter.stop();
                    rpmSelector.clearShot();
                    intake.stop();
                    transfer.stopTransfer();
                }, shooter, intake, transfer));
    }

    //DPAD RIGHT RUN INTAKE ROLLERs, 
    //DPAD LEFT STOP INTAKE ROLLERS,
    //DPAD RIGHT + RIGHT BUMPER RUN Nectar ROllers
    //DPAD LEFT + RIGHT BUMPER EJECT INTAKE ROLLERS, DPAD LEFT + LEFT BUMPER EJECT INTAKE ROLLERS
    private void bindIntakeControls(GamepadEx gamepad, IntakeSubsystem intake, TransferSubsystem transfer) {
        // D-pad Right selects pollen by default; bumper combos select nectar or flower.
        new GamepadButton(gamepad, GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(new InstantCommand(() -> {
                    Runnable startRollers;
                    if (gamepad.getButton(GamepadKeys.Button.RIGHT_BUMPER)) {
                        startRollers = intake::runRollersNectar;
                    } else if (gamepad.getButton(GamepadKeys.Button.LEFT_BUMPER)) {
                        startRollers = intake::runRollersFlower;
                    } else {
                        startRollers = intake::runRollersPollen;
                    }
                    new IntakeCommand(intake, transfer, startRollers).schedule();
                }));

        // D-pad Left stops the intake. Adding Right Bumper reverses it, in either press order.
        Trigger left = new GamepadButton(gamepad, GamepadKeys.Button.DPAD_LEFT);
        Trigger rightBumper = new GamepadButton(gamepad, GamepadKeys.Button.RIGHT_BUMPER);
        left.and(rightBumper.negate())
                .whenActive(new InstantCommand(intake::stop, intake));
        EjectIntakeCommand eject = new EjectIntakeCommand(intake);
        left.and(rightBumper)
                .whileActiveOnce(eject);
    }

    //DPAD UP/DOWN shooter RPM controls
    private void bindShooterRPMControls(GamepadEx gamepad, DistanceShooterRPM rpmSelector) {
        // Target changes must not interrupt the command regulating or firing the shooter.
        new GamepadButton(gamepad, GamepadKeys.Button.DPAD_UP)
                .whenPressed(new InstantCommand(() -> {
                    if (!gamepad.getButton(GamepadKeys.Button.DPAD_LEFT)
                            && !gamepad.getButton(GamepadKeys.Button.DPAD_RIGHT)
                            && !gamepad.getButton(GamepadKeys.Button.LEFT_BUMPER)
                            && !gamepad.getButton(GamepadKeys.Button.RIGHT_BUMPER)) {
                        rpmSelector.adjustManualRPM(50.0);
                    }
                }));
        new GamepadButton(gamepad, GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new InstantCommand(() -> {
                    if (!gamepad.getButton(GamepadKeys.Button.DPAD_LEFT)
                            && !gamepad.getButton(GamepadKeys.Button.DPAD_RIGHT)
                            && !gamepad.getButton(GamepadKeys.Button.LEFT_BUMPER)
                            && !gamepad.getButton(GamepadKeys.Button.RIGHT_BUMPER)) {
                        rpmSelector.adjustManualRPM(-50.0);
                    }
                }));
    }

    // DPAD UP/DOWN + LEFT BUMPER intake speed controls
    private void bindIntakeSpeedControls(GamepadEx gamepad, IntakeSubsystem intake) {
        // Right Bumper + Up increases intake power. Left Bumper + Down decreases it.

        // Left or Right D-pad takes priority because those buttons select an intake action.
        new GamepadButton(gamepad, GamepadKeys.Button.DPAD_UP)
                .whenPressed(new InstantCommand(() -> {
                    if (gamepad.getButton(GamepadKeys.Button.LEFT_BUMPER)
                            && !gamepad.getButton(GamepadKeys.Button.DPAD_RIGHT)
                            && !gamepad.getButton(GamepadKeys.Button.DPAD_LEFT)) {
                        intake.increasePower();
                    }
                }));
        new GamepadButton(gamepad, GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new InstantCommand(() -> {
                    if (gamepad.getButton(GamepadKeys.Button.LEFT_BUMPER)
                            && !gamepad.getButton(GamepadKeys.Button.DPAD_RIGHT)
                            && !gamepad.getButton(GamepadKeys.Button.DPAD_LEFT)) {
                        intake.decreasePower();
                    }
                }));
    }

    //DPAD UP/DOWN + RIGHT BUMPER transfer speed controls
    private void bindTransferSpeedControls(GamepadEx gamepad, TransferSubsystem transfer) {
        // Right Bumper + Up increases transfer power. Left Bumper + Down decreases it.

        // Left or Right D-pad takes priority because those buttons select a transfer action.
        new GamepadButton(gamepad, GamepadKeys.Button.DPAD_UP)
                .whenPressed(new InstantCommand(() -> {
                    if (gamepad.getButton(GamepadKeys.Button.RIGHT_BUMPER)
                            && !gamepad.getButton(GamepadKeys.Button.DPAD_RIGHT)
                            && !gamepad.getButton(GamepadKeys.Button.DPAD_LEFT)) {
                        transfer.increaseRPM();
                    }
                }));
        new GamepadButton(gamepad, GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new InstantCommand(() -> {
                    if (gamepad.getButton(GamepadKeys.Button.RIGHT_BUMPER)
                            && !gamepad.getButton(GamepadKeys.Button.DPAD_RIGHT)
                            && !gamepad.getButton(GamepadKeys.Button.DPAD_LEFT)) {
                        transfer.decreaseRPM();
                    }
                }));
    }

}
