package org.firstinspires.ftc.teamcode.lib.GamepadDriver;

import java.util.function.Supplier;

public interface GamepadInput<T> extends Supplier<T> {
    default void update(){};
}
