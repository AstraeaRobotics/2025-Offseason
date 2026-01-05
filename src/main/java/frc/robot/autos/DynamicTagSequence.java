package frc.robot.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.vision.AlignFull;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class DynamicTagSequence extends SequentialCommandGroup {
  
  // Tag positions from your DriveToTag.java
  private static final double[][] TAG_POSITIONS = {
    {1.544, 0.856, -90},   // Tag 0
    {1.32, 1.178, 90},   // Tag 1
    {2.09, 1.12, 90},    // Tag 2
    {3.179, 1.30, 90},   // Tag 3
    {3.587, 1, 0},     // Tag 4
    {2.42, 0.81, -56.9}    // Tag 5
  };
  
  private static final double[] HOME = {0.285, 0.43, 0};
  
  private static final PathConstraints CONSTRAINTS = new PathConstraints(
    0.5, 
    0.4,
    Units.degreesToRadians(90),
    Units.degreesToRadians(180)
  );

  public DynamicTagSequence(
      SwerveSubsystem swerveSubsystem, 
      VisionSubsystem visionSubsystem,
      int tag1, 
      int tag2) {
    
    // Go to first tag
    if (isValidTag(tag1)) {
      addCommands(
        createPathToTag(tag1),
        new WaitCommand(0.2),
        new AlignFull(visionSubsystem, swerveSubsystem, true).withTimeout(2.0)
      );
    }
    
    // Go to second tag
    if (isValidTag(tag2)) {
      addCommands(
        createPathToTag(tag2),
        new WaitCommand(0.2),
        new AlignFull(visionSubsystem, swerveSubsystem, true).withTimeout(2.0)
      );
    }
    
    // Return home
    addCommands(
      AutoBuilder.pathfindToPose(
        new Pose2d(HOME[0], HOME[1], Rotation2d.fromDegrees(HOME[2])),
        CONSTRAINTS,
        0.0
      )
    );
    
    addRequirements(swerveSubsystem, visionSubsystem);
  }
  
  private static boolean isValidTag(int tagID) {
    return tagID >= 0 && tagID <= 5;
  }
  
  /**
   * Creates a command to pathfind to the specified tag
   */
  private static Command createPathToTag(int tagID) {
    double[] pos = TAG_POSITIONS[tagID];
    return AutoBuilder.pathfindToPose(
      new Pose2d(pos[0], pos[1], Rotation2d.fromDegrees(pos[2])),
      CONSTRAINTS,
      0.0
    );
  }
  
  public static boolean areTagsValid(int tag1, int tag2) {
    return isValidTag(tag1) && isValidTag(tag2);
  }
}