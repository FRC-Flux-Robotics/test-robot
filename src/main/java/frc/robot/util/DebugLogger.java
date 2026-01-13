// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Lightweight debug logging utility with configurable log levels.
 *
 * <p>Outputs to console for development debugging. Log levels can be changed at runtime via
 * SmartDashboard or programmatically.
 *
 * <p>Usage:
 *
 * <pre>{@code
 * DebugLogger.info("Drivetrain", "Initialized successfully");
 * DebugLogger.debug("Drivetrain", "Pose: " + pose);
 * DebugLogger.warn("Vision", "Camera disconnected");
 * DebugLogger.error("Safety", "Emergency stop triggered");
 * }</pre>
 */
public final class DebugLogger {

    /** Log levels in order of severity. */
    public enum Level {
        DEBUG(0),
        INFO(1),
        WARN(2),
        ERROR(3),
        OFF(4);

        private final int severity;

        Level(int severity) {
            this.severity = severity;
        }

        int getSeverity() {
            return severity;
        }
    }

    private static final String DASHBOARD_KEY = "DebugLogLevel";
    private static final long START_TIME_US = RobotController.getFPGATime();

    private static Level currentLevel = Level.INFO;
    private static boolean enabled = true;

    private DebugLogger() {
        // Utility class - prevent instantiation
    }

    /**
     * Log a debug message. Use for verbose development output.
     *
     * @param tag Component or subsystem name
     * @param message Message to log
     */
    public static void debug(String tag, String message) {
        log(Level.DEBUG, tag, message);
    }

    /**
     * Log an info message. Use for normal operational events.
     *
     * @param tag Component or subsystem name
     * @param message Message to log
     */
    public static void info(String tag, String message) {
        log(Level.INFO, tag, message);
    }

    /**
     * Log a warning message. Use for recoverable problems.
     *
     * @param tag Component or subsystem name
     * @param message Message to log
     */
    public static void warn(String tag, String message) {
        log(Level.WARN, tag, message);
        DriverStation.reportWarning("[" + tag + "] " + message, false);
    }

    /**
     * Log an error message. Use for serious failures.
     *
     * @param tag Component or subsystem name
     * @param message Message to log
     */
    public static void error(String tag, String message) {
        log(Level.ERROR, tag, message);
        DriverStation.reportError("[" + tag + "] " + message, false);
    }

    /**
     * Set the current log level. Messages below this level will be filtered.
     *
     * @param level The minimum level to log
     */
    public static void setLevel(Level level) {
        currentLevel = level;
        SmartDashboard.putString(DASHBOARD_KEY, level.name());
    }

    /**
     * Get the current log level.
     *
     * @return The current log level
     */
    public static Level getLevel() {
        return currentLevel;
    }

    /**
     * Enable or disable all logging.
     *
     * @param enable true to enable logging, false to disable
     */
    public static void setEnabled(boolean enable) {
        enabled = enable;
    }

    /**
     * Check if logging is enabled.
     *
     * @return true if logging is enabled
     */
    public static boolean isEnabled() {
        return enabled;
    }

    /**
     * Update the log level from SmartDashboard. Call this periodically if you want runtime control.
     */
    public static void updateFromDashboard() {
        String levelStr = SmartDashboard.getString(DASHBOARD_KEY, currentLevel.name());
        try {
            Level newLevel = Level.valueOf(levelStr.toUpperCase());
            if (newLevel != currentLevel) {
                currentLevel = newLevel;
            }
        } catch (IllegalArgumentException e) {
            // Invalid level string - ignore
        }
    }

    /**
     * Initialize the logger by publishing the current level to SmartDashboard.
     */
    public static void initialize() {
        SmartDashboard.putString(DASHBOARD_KEY, currentLevel.name());
    }

    private static void log(Level level, String tag, String message) {
        if (!enabled || level.getSeverity() < currentLevel.getSeverity()) {
            return;
        }

        if (tag == null) {
            tag = "Unknown";
        }
        if (message == null) {
            message = "";
        }

        double elapsedSeconds = (RobotController.getFPGATime() - START_TIME_US) / 1_000_000.0;
        String timestamp = String.format("%.3f", elapsedSeconds);
        String levelStr = level.name();

        System.out.println("[" + timestamp + "] [" + levelStr + "] [" + tag + "] " + message);
    }
}
