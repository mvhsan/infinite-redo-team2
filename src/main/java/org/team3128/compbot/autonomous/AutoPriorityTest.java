package org.team3128.compbot.autonomous;

import edu.wpi.first.wpilibj.command.CommandGroup;

import org.team3128.common.utility.math.Pose2D;
import org.team3128.common.utility.math.Rotation2D;
import org.team3128.common.drive.Drive;
import org.team3128.common.hardware.gyroscope.Gyro;
import org.team3128.common.hardware.limelight.Limelight;
import org.team3128.common.drive.DriveCommandRunning;
import org.team3128.compbot.autonomous.*;
import org.team3128.compbot.commands.*;
import org.team3128.compbot.subsystems.Constants.VisionConstants;
import org.team3128.compbot.subsystems.*;
import org.team3128.compbot.subsystems.RobotTracker;

public class AutoPriorityTest extends CommandGroup {

    RobotTracker robotTracker = RobotTracker.getInstance();

    public AutoPriorityTest(FalconDrive drive, double timeoutMs) {       
        addSequential(new CmdAutoTrajectory(drive, 120, 0.5, timeoutMs, 
            new Pose2D(0, 0, Rotation2D.fromDegrees(0)),
            new Pose2D(194 * Constants.MechanismConstants.inchesToMeters, 27 * Constants.MechanismConstants.inchesToMeters, Rotation2D.fromDegrees(robotTracker.getOdometry().getRotation().getDegrees() + 180)))); // 194.63 inches length and 27.75 inches width
        }
}