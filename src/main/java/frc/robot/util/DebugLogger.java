// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;

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
    private static final DateTimeFormatter FILE_NAME_FORMATTER =
            DateTimeFormatter.ofPattern("yyyyMMdd_HHmmss");

    private static Level currentLevel = Level.INFO;
    private static boolean enabled = true;
    private static PrintWriter fileWriter = null;
    private static String currentLogFile = null;

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

    /**
     * Enable file-based logging to the specified directory.
     *
     * <p>Creates a new log file with timestamp in the filename. If the directory does not exist, it
     * will be created. If file creation fails, logging continues to console only.
     *
     * @param directory The directory path where log files should be created
     * @return true if file logging was enabled successfully, false otherwise
     */
    public static boolean enableFileLogging(String directory) {
        if (fileWriter != null) {
            // Already enabled, close existing writer first
            disableFileLogging();
        }

        try {
            File logDir = new File(directory);
            if (!logDir.exists() && !logDir.mkdirs()) {
                System.err.println("[DebugLogger] Failed to create log directory: " + directory);
                return false;
            }

            String timestamp = LocalDateTime.now().format(FILE_NAME_FORMATTER);
            String filename = "debug_" + timestamp + ".log";
            File logFile = new File(logDir, filename);
            currentLogFile = logFile.getAbsolutePath();

            fileWriter = new PrintWriter(new FileWriter(logFile, true), true);
            info("DebugLogger", "File logging enabled: " + currentLogFile);
            return true;
        } catch (IOException e) {
            System.err.println("[DebugLogger] Failed to create log file: " + e.getMessage());
            fileWriter = null;
            currentLogFile = null;
            return false;
        }
    }

    /**
     * Disable file-based logging and close the log file.
     */
    public static void disableFileLogging() {
        if (fileWriter != null) {
            info("DebugLogger", "File logging disabled");
            fileWriter.close();
            fileWriter = null;
            currentLogFile = null;
        }
    }

    /**
     * Check if file logging is currently enabled.
     *
     * @return true if file logging is enabled
     */
    public static boolean isFileLoggingEnabled() {
        return fileWriter != null;
    }

    /**
     * Get the path to the current log file.
     *
     * @return The absolute path to the current log file, or null if file logging is disabled
     */
    public static String getCurrentLogFile() {
        return currentLogFile;
    }

    /**
     * Flush any buffered log data to file. Call this periodically or before shutdown.
     */
    public static void flush() {
        if (fileWriter != null) {
            fileWriter.flush();
        }
    }

    /**
     * Close the logger, releasing any resources. Should be called on robot shutdown.
     */
    public static void close() {
        disableFileLogging();
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

        String formattedMessage = "[" + timestamp + "] [" + levelStr + "] [" + tag + "] " + message;

        // Console output
        System.out.println(formattedMessage);

        // File output (if enabled)
        if (fileWriter != null) {
            fileWriter.println(formattedMessage);
        }
    }
}
