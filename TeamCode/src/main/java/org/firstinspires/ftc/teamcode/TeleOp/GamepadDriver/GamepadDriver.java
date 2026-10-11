package org.firstinspires.ftc.teamcode.TeleOp.GamepadDriver;

import com.seattlesolvers.solverslib.command.CommandBase;

import java.lang.reflect.Field;
import java.util.Arrays;

public abstract class GamepadDriver {
    public class Update extends CommandBase {
        private GamepadInput<?>[] updatableInputs;

        @Override
        public void initialize() {
            // Inspect fields of the outer subclass (e.g. SampleGamepad)
            Field[] fields = GamepadDriver.this.getClass().getDeclaredFields();
            updatableInputs = Arrays.stream(fields)
                    .filter(field -> GamepadInput.class.isAssignableFrom(field.getType()))
                    .map(field -> {
                        try {
                            field.setAccessible(true);
                            return (GamepadInput<?>) field.get(GamepadDriver.this);
                        } catch (IllegalAccessException e) {
                            throw new RuntimeException(e);
                        }
                    })
                    .toArray(GamepadInput<?>[]::new);
        }

        @Override
        public void execute() {
            if (updatableInputs != null) {
                Arrays.stream(updatableInputs).forEach(GamepadInput::update);
            }
        }

        @Override
        public boolean isFinished() {
            return false; // Continuous background command
        }
    }
}
