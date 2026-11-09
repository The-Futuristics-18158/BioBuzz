package org.firstinspires.ftc.teamcode.Utility;

/**
 * Enumeration of available vision processing modes for the camera subsystems.
 *
 * @author kw126
 * @author Glenn
 * @author knutt5
 */
public enum VisionProcessorMode {
    /** No active vision processors; camera is idle or raw preview only. */
    NONE,
    /** Vision processor configured to detect purple and green game artifacts. */
    ARTIFACT
}