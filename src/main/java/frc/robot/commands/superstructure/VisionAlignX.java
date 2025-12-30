package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RobotSuperstructure;
import frc.robot.subsystems.RobotSuperstructure.SuperstructureState;

public class VisionAlignX extends Command {
    private final RobotSuperstructure m_superstructure;
    private final boolean m_slowMode;
    
    public VisionAlignX(RobotSuperstructure superstructure, boolean slowMode) {
        m_superstructure = superstructure;
        m_slowMode = slowMode;
        addRequirements(superstructure);
    }
    
    @Override
    public void initialize() {
        m_superstructure.setState(SuperstructureState.VISION_ALIGN_X);
    }
    
    @Override
    public void execute() {
        m_superstructure.executeVisionAlignX(m_slowMode);
    }
    
    @Override
    public void end(boolean interrupted) {
        m_superstructure.stopMovement();
        m_superstructure.setState(SuperstructureState.MANUAL_DRIVE);
    }
    
    @Override
    public boolean isFinished() {
        return m_superstructure.isVisionAlignedX();
    }
}