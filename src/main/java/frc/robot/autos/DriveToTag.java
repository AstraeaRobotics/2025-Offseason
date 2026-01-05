package frc.robot.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.vision.AlignFull;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class DriveToTag extends SequentialCommandGroup {
  
  private static final PathConstraints CONSTRAINTS = new PathConstraints(
    0.5, 
    0.4,
    Units.degreesToRadians(90),
    Units.degreesToRadians(180) 
  );

  private static final double[][] TAG_POSITIONS = {
    {1.544, 0.856, -90},  // Tag 0
    {1.32, 1.178, 90},  // Tag 1
    {2.09, 1.12, 90},    // Tag 2 
    {3.179, 1.30, 90},     // Tag 3 
    {3.587, 1, 0},    // Tag 4 
    {2.42, 0.81, -56.9}       // Tag 5 
  };

  public DriveToTag(
      SwerveSubsystem swerveSubsystem, 
      VisionSubsystem visionSubsystem,
      int tagID) {
    
    if (tagID < 0 || tagID >= TAG_POSITIONS.length) {
      System.err.println("Invalid tag ID: " + tagID);
      return;
    }

    double[] position = TAG_POSITIONS[tagID];
    double x = position[0];
    double y = position[1];
    double rotation = position[2];

    addCommands(
      AutoBuilder.pathfindToPose(
        new Pose2d(x, y, Rotation2d.fromDegrees(rotation)), 
        CONSTRAINTS,
        0.0
      ),

      new WaitCommand(.2),

      new AlignFull(visionSubsystem, swerveSubsystem, true)
    );
    
    addRequirements(swerveSubsystem, visionSubsystem);
  }
}