package org.firstinspires.ftc.teamcode.TeleOp.UniversalTools;

/**
 * Processes normalized robot-centric forward, left-strafe and CCW-turn inputs.
 * Each axis is independent. Rates are commanded power units per second, not in/s^2.
 * Call update once per control loop with elapsed seconds; call reset before driving.
 */
public final class JoystickDriveProcessor {
    public static final class JoystickResponseConfig {
        public final double deadzone;
        public final double exponent;
        public final double acceleration;
        public final double deceleration;

        /** exponent=1 is linear; >1 gives finer control near center. Rates must be positive. */
        public JoystickResponseConfig(double deadzone, double exponent,
                          double acceleration, double deceleration) {
            if (!Double.isFinite(deadzone) || deadzone < 0 || deadzone >= 1
                    || !Double.isFinite(exponent) || exponent < 1
                    || !Double.isFinite(acceleration) || acceleration <= 0
                    || !Double.isFinite(deceleration) || deceleration <= 0) {
                throw new IllegalArgumentException("Invalid joystick axis configuration");
            }
            this.deadzone = deadzone;
            this.exponent = exponent;
            this.acceleration = acceleration;
            this.deceleration = deceleration;
        }
    }

    public static final class Output {
        public final double forward;
        public final double strafe;
        public final double turn;

        private Output(double forward, double strafe, double turn) {
            this.forward = forward;
            this.strafe = strafe;
            this.turn = turn;
        }
    }

    private final JoystickResponseConfig moveResponseConfig;
    private final JoystickResponseConfig turnResponseConfig;
    private double forward;
    private double strafe;
    private double turn;

    public JoystickDriveProcessor(JoystickResponseConfig moveResponseConfig, JoystickResponseConfig turnResponseConfig) {
        if (moveResponseConfig == null || turnResponseConfig == null) {
            throw new IllegalArgumentException("Axis configurations must not be null");
        }
        this.moveResponseConfig = moveResponseConfig;
        this.turnResponseConfig = turnResponseConfig;
    }

    public Output update(double forwardInput, double strafeInput,
                         double turnInput, double elapsedSeconds) {
        if (!Double.isFinite(elapsedSeconds) || elapsedSeconds < 0) {
            throw new IllegalArgumentException("Elapsed time must be finite and nonnegative");
        }
        // Avoid a large power jump after a stalled loop. Normal loop timing is unaffected.
        double dt = Math.min(elapsedSeconds, 0.05);
        forward = limit(forward, shape(forwardInput, moveResponseConfig), moveResponseConfig, dt);
        strafe = limit(strafe, shape(strafeInput, moveResponseConfig), moveResponseConfig, dt);
        turn = limit(turn, shape(turnInput, turnResponseConfig), turnResponseConfig, dt);
        return new Output(forward, strafe, turn);
    }

    /** Immediately clears stored power; emergency stopping should bypass the ramp. */
    public void reset() {
        forward = 0;
        strafe = 0;
        turn = 0;
    }

    private static double shape(double input, JoystickResponseConfig config) {
        if (!Double.isFinite(input)) {
            throw new IllegalArgumentException("Joystick input must be finite");
        }
        double magnitude = Math.min(Math.abs(input), 1.0);
        if (magnitude <= config.deadzone) return 0;
        // Rescale the remaining travel so the deadzone boundary starts smoothly at zero.
        double scaled = (magnitude - config.deadzone) / (1.0 - config.deadzone);
        // To reduce the computional heavy calculation.
        double curved;
        if (config.exponent == 1.0) {
            curved = scaled;
        } else if (config.exponent == 2.0) {
            curved = scaled * scaled;
        } else {
            curved = Math.pow(scaled, config.exponent);
        }
        return Math.copySign(curved, input);
    }

    private static double limit(double current, double target, JoystickResponseConfig config, double dt) {
        // current: previous loop's commanded power; target: desired power after deadzone and response curve.
        if (current * target < 0) {
            // Reverse by decelerating to zero first, then accelerating in the new direction.
            double timeToZero = Math.abs(current) / config.deceleration;
            if (dt <= timeToZero) {
                return approach(current, 0, config.deceleration * dt);
            }
            return approach(0, target, config.acceleration * (dt - timeToZero));
        }
        double rate = Math.abs(target) > Math.abs(current)
                ? config.acceleration : config.deceleration;
        return approach(current, target, rate * dt);
    }

    private static double approach(double current, double target, double maxChange) {
        return current + Math.max(-maxChange, Math.min(maxChange, target - current));
    }
}
