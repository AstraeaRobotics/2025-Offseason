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

public class DriveToPoseAlign extends SequentialCommandGroup {
  
  private static final PathConstraints CONSTRAINTS = new PathConstraints(
    0.5, 
    0.4,
    Units.degreesToRadians(90),
    Units.degreesToRadians(180) 
  );

  public DriveToPoseAlign(
      SwerveSubsystem swerveSubsystem, 
      VisionSubsystem visionSubsystem,
      double x,
      double y, 
      double rotationDegrees) {
    
    addCommands(
      AutoBuilder.pathfindToPose(
        new Pose2d(x, y, Rotation2d.fromDegrees(rotationDegrees)), 
        CONSTRAINTS,
        0.0
      ),
      new WaitCommand(0.5),
      
      new AlignFull(visionSubsystem, swerveSubsystem, true)
    );
    
    addRequirements(swerveSubsystem, visionSubsystem);
  }
}