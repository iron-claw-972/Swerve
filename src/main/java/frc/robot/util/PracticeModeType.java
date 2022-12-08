package frc.robot.util;

public enum PracticeModeType {
    NONE, HEADING_PID_TUNE;

    public String toString() {
        switch (this) {
            case NONE:
                return "NONE";
            case HEADING_PID_TUNE:
                return "HEADING_PID_TUNE";
            default:
                return "NONE";
        }
    }
}
