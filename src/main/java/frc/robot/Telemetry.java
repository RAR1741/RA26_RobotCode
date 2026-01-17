package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Telemetry {

    private static boolean enabled = true;

    public static void setEnabled(boolean value) {
        enabled = value;
    }

    public static void log(String message) {
        if (!enabled) return;

        System.out.println("[LOG " + timestamp() + "] " + message);
    }

    /** Log a number to SmartDashboard. */
    public static void logNumber(String key, double value) {
        if (!enabled) return;

        SmartDashboard.putNumber(key, value);
    }

    /** Log a string to SmartDashboard. */
    public static void logString(String key, String value) {
        if (!enabled) return;

        SmartDashboard.putString(key, value);
    }

    /** Timestamp helper. */
    private static String timestamp() {
        return String.format("%.3f", Timer.getFPGATimestamp());
    }
}
