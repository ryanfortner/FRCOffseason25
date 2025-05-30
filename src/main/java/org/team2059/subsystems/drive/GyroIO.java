package org.team2059.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
    @AutoLog
    public static class GyroIOInputs {
        public boolean connected = false;
        public double yaw = 0;
        public boolean is180Rotated = false;
    }

    default public void updateInputs(GyroIOInputs inputs) {}

    default public void reset() {}

    default public void set180Rotation(boolean enabled) {}
}
