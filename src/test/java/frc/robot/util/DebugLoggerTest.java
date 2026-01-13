// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import static org.junit.jupiter.api.Assertions.*;

import frc.robot.util.DebugLogger.Level;
import java.io.ByteArrayOutputStream;
import java.io.File;
import java.io.IOException;
import java.io.PrintStream;
import java.nio.file.Files;
import java.nio.file.Path;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

class DebugLoggerTest {

    private final ByteArrayOutputStream outContent = new ByteArrayOutputStream();
    private final PrintStream originalOut = System.out;

    @BeforeEach
    void setUp() {
        System.setOut(new PrintStream(outContent));
        DebugLogger.setEnabled(true);
        DebugLogger.setLevel(Level.DEBUG); // Start with most verbose level
    }

    @AfterEach
    void tearDown() {
        System.setOut(originalOut);
        DebugLogger.setEnabled(true);
        DebugLogger.setLevel(Level.INFO); // Reset to default
        DebugLogger.disableFileLogging(); // Ensure file logging is disabled
    }

    @Test
    void debugMessageIsLoggedAtDebugLevel() {
        DebugLogger.setLevel(Level.DEBUG);
        DebugLogger.debug("Test", "Debug message");

        String output = outContent.toString();
        assertTrue(output.contains("[DEBUG]"), "Output should contain DEBUG level");
        assertTrue(output.contains("[Test]"), "Output should contain tag");
        assertTrue(output.contains("Debug message"), "Output should contain message");
    }

    @Test
    void infoMessageIsLoggedAtInfoLevel() {
        DebugLogger.setLevel(Level.INFO);
        DebugLogger.info("Test", "Info message");

        String output = outContent.toString();
        assertTrue(output.contains("[INFO]"), "Output should contain INFO level");
        assertTrue(output.contains("Info message"), "Output should contain message");
    }

    @Test
    void warnMessageIsLoggedAtWarnLevel() {
        DebugLogger.setLevel(Level.WARN);
        DebugLogger.warn("Test", "Warn message");

        String output = outContent.toString();
        assertTrue(output.contains("[WARN]"), "Output should contain WARN level");
        assertTrue(output.contains("Warn message"), "Output should contain message");
    }

    @Test
    void errorMessageIsLoggedAtErrorLevel() {
        DebugLogger.setLevel(Level.ERROR);
        DebugLogger.error("Test", "Error message");

        String output = outContent.toString();
        assertTrue(output.contains("[ERROR]"), "Output should contain ERROR level");
        assertTrue(output.contains("Error message"), "Output should contain message");
    }

    @Test
    void debugMessageIsFilteredAtInfoLevel() {
        DebugLogger.setLevel(Level.INFO);
        DebugLogger.debug("Test", "Should not appear");

        String output = outContent.toString();
        assertFalse(output.contains("Should not appear"), "DEBUG should be filtered at INFO level");
    }

    @Test
    void infoMessageIsFilteredAtWarnLevel() {
        DebugLogger.setLevel(Level.WARN);
        DebugLogger.info("Test", "Should not appear");

        String output = outContent.toString();
        assertFalse(output.contains("Should not appear"), "INFO should be filtered at WARN level");
    }

    @Test
    void warnMessageIsFilteredAtErrorLevel() {
        DebugLogger.setLevel(Level.ERROR);
        DebugLogger.warn("Test", "Should not appear");

        String output = outContent.toString();
        assertFalse(output.contains("Should not appear"), "WARN should be filtered at ERROR level");
    }

    @Test
    void offLevelDisablesAllLogging() {
        DebugLogger.setLevel(Level.OFF);

        DebugLogger.debug("Test", "Debug");
        DebugLogger.info("Test", "Info");
        DebugLogger.warn("Test", "Warn");
        DebugLogger.error("Test", "Error");

        String output = outContent.toString();
        assertTrue(output.isEmpty(), "No messages should be logged at OFF level");
    }

    @Test
    void setEnabledFalseDisablesLogging() {
        DebugLogger.setEnabled(false);
        DebugLogger.setLevel(Level.DEBUG);

        DebugLogger.debug("Test", "Should not appear");
        DebugLogger.info("Test", "Should not appear");

        String output = outContent.toString();
        assertTrue(output.isEmpty(), "No messages should be logged when disabled");
    }

    @Test
    void setEnabledTrueEnablesLogging() {
        DebugLogger.setEnabled(false);
        DebugLogger.setEnabled(true);
        DebugLogger.setLevel(Level.DEBUG);

        DebugLogger.debug("Test", "Should appear");

        String output = outContent.toString();
        assertTrue(output.contains("Should appear"), "Message should appear after re-enabling");
    }

    @Test
    void getLevelReturnsCurrentLevel() {
        DebugLogger.setLevel(Level.WARN);
        assertEquals(Level.WARN, DebugLogger.getLevel());

        DebugLogger.setLevel(Level.DEBUG);
        assertEquals(Level.DEBUG, DebugLogger.getLevel());
    }

    @Test
    void isEnabledReturnsEnabledState() {
        DebugLogger.setEnabled(true);
        assertTrue(DebugLogger.isEnabled());

        DebugLogger.setEnabled(false);
        assertFalse(DebugLogger.isEnabled());
    }

    @Test
    void nullTagIsHandled() {
        DebugLogger.setLevel(Level.DEBUG);
        DebugLogger.debug(null, "Message with null tag");

        String output = outContent.toString();
        assertTrue(output.contains("[Unknown]"), "Null tag should be replaced with Unknown");
        assertTrue(output.contains("Message with null tag"), "Message should still appear");
    }

    @Test
    void nullMessageIsHandled() {
        DebugLogger.setLevel(Level.DEBUG);
        DebugLogger.debug("Test", null);

        String output = outContent.toString();
        assertTrue(output.contains("[Test]"), "Tag should appear");
        assertTrue(output.contains("[DEBUG]"), "Level should appear");
    }

    @Test
    void outputContainsTimestamp() {
        DebugLogger.setLevel(Level.DEBUG);
        DebugLogger.debug("Test", "Message");

        String output = outContent.toString();
        // Timestamp format is [X.XXX] - may be negative in tests due to FPGA time initialization
        assertTrue(
                output.matches("(?s)\\[-?\\d+\\.\\d{3}\\].*"),
                "Output should start with timestamp, got: " + output);
    }

    @Test
    void levelSeverityOrderIsCorrect() {
        assertTrue(Level.DEBUG.getSeverity() < Level.INFO.getSeverity());
        assertTrue(Level.INFO.getSeverity() < Level.WARN.getSeverity());
        assertTrue(Level.WARN.getSeverity() < Level.ERROR.getSeverity());
        assertTrue(Level.ERROR.getSeverity() < Level.OFF.getSeverity());
    }

    // File logging tests

    @TempDir
    Path tempDir;

    @Test
    void enableFileLoggingCreatesLogFile() {
        boolean enabled = DebugLogger.enableFileLogging(tempDir.toString());

        assertTrue(enabled, "File logging should be enabled successfully");
        assertTrue(DebugLogger.isFileLoggingEnabled(), "isFileLoggingEnabled should return true");
        assertNotNull(DebugLogger.getCurrentLogFile(), "getCurrentLogFile should return path");

        File logFile = new File(DebugLogger.getCurrentLogFile());
        assertTrue(logFile.exists(), "Log file should exist");
        assertTrue(logFile.getName().startsWith("debug_"), "Log file should start with debug_");
        assertTrue(logFile.getName().endsWith(".log"), "Log file should end with .log");
    }

    @Test
    void disableFileLoggingClosesFile() {
        DebugLogger.enableFileLogging(tempDir.toString());
        String logFilePath = DebugLogger.getCurrentLogFile();

        DebugLogger.disableFileLogging();

        assertFalse(DebugLogger.isFileLoggingEnabled(), "isFileLoggingEnabled should return false");
        assertNull(DebugLogger.getCurrentLogFile(), "getCurrentLogFile should return null");

        // File should still exist after closing
        File logFile = new File(logFilePath);
        assertTrue(logFile.exists(), "Log file should still exist after closing");
    }

    @Test
    void messagesAreWrittenToFile() throws IOException {
        DebugLogger.enableFileLogging(tempDir.toString());
        String logFilePath = DebugLogger.getCurrentLogFile();

        DebugLogger.info("FileTest", "Test message for file");
        DebugLogger.flush();

        String content = Files.readString(Path.of(logFilePath));
        assertTrue(content.contains("[INFO]"), "File should contain INFO level");
        assertTrue(content.contains("[FileTest]"), "File should contain tag");
        assertTrue(content.contains("Test message for file"), "File should contain message");
    }

    @Test
    void multipleMessagesAreWrittenToFile() throws IOException {
        DebugLogger.enableFileLogging(tempDir.toString());
        String logFilePath = DebugLogger.getCurrentLogFile();

        DebugLogger.debug("Test", "Debug message");
        DebugLogger.info("Test", "Info message");
        DebugLogger.warn("Test", "Warn message");
        DebugLogger.error("Test", "Error message");
        DebugLogger.flush();

        String content = Files.readString(Path.of(logFilePath));
        assertTrue(content.contains("[DEBUG]"), "File should contain DEBUG");
        assertTrue(content.contains("[INFO]"), "File should contain INFO");
        assertTrue(content.contains("[WARN]"), "File should contain WARN");
        assertTrue(content.contains("[ERROR]"), "File should contain ERROR");
    }

    @Test
    void fileLoggingCreatesDirectoryIfNeeded() {
        Path nestedDir = tempDir.resolve("nested/logs/dir");

        boolean enabled = DebugLogger.enableFileLogging(nestedDir.toString());

        assertTrue(enabled, "File logging should succeed with nested directory");
        assertTrue(Files.exists(nestedDir), "Nested directory should be created");
    }

    @Test
    void closeDisablesFileLogging() {
        DebugLogger.enableFileLogging(tempDir.toString());
        assertTrue(DebugLogger.isFileLoggingEnabled());

        DebugLogger.close();

        assertFalse(DebugLogger.isFileLoggingEnabled());
        assertNull(DebugLogger.getCurrentLogFile());
    }

    @Test
    void isFileLoggingEnabledReturnsFalseWhenNotEnabled() {
        assertFalse(DebugLogger.isFileLoggingEnabled());
        assertNull(DebugLogger.getCurrentLogFile());
    }

    @Test
    void enableFileLoggingTwiceClosesFirstFile() throws IOException {
        DebugLogger.enableFileLogging(tempDir.toString());
        String firstFile = DebugLogger.getCurrentLogFile();
        DebugLogger.info("Test", "First file message");
        DebugLogger.flush();

        // Small delay to ensure different timestamp
        try {
            Thread.sleep(1100);
        } catch (InterruptedException e) {
            // Ignore
        }

        DebugLogger.enableFileLogging(tempDir.toString());
        String secondFile = DebugLogger.getCurrentLogFile();
        DebugLogger.info("Test", "Second file message");
        DebugLogger.flush();

        assertNotEquals(firstFile, secondFile, "Second file should have different path");

        String firstContent = Files.readString(Path.of(firstFile));
        String secondContent = Files.readString(Path.of(secondFile));

        assertTrue(firstContent.contains("First file message"), "First file should have first message");
        assertTrue(secondContent.contains("Second file message"), "Second file should have second message");
    }

    @Test
    void flushDoesNothingWhenFileLoggingDisabled() {
        // Should not throw
        DebugLogger.flush();
    }
}
