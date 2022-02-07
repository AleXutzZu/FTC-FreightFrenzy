package org.firstinspires.ftc.teamcode.util;

/**
 * PID Controller interface.
 * Provides an update() function
 */
@FunctionalInterface
public interface Driveable {
    double update(double yaw);
}
