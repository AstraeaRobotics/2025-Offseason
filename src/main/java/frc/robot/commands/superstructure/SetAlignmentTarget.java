package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants.AlignmentPosition;
import frc.robot.subsystems.RobotSuperstructure;

public class SetAlignmentTarget extends Command {
    private final RobotSuperstructure m_superstructure;
    private final AlignmentPosition m_position;
    
    public SetAlignmentTarget(RobotSuperstructure superstructure, AlignmentPosition position) {
        m_superstructure = superstructure;
        m_position = position;
    }
    
    @Override
    public void initialize() {
        m_superstructure.setAlignmentTarget(m_position);
    }
    
    @Override
    public boolean isFinished() {
        return true;
    }
}