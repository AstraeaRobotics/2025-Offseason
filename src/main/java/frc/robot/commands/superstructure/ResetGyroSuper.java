package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RobotSuperstructure;

public class ResetGyroSuper extends Command {
    private final RobotSuperstructure m_superstructure;
    
    public ResetGyroSuper(RobotSuperstructure superstructure) {
        m_superstructure = superstructure;
    }
    
    @Override
    public void initialize() {
        m_superstructure.resetGyro();
    }
    
    @Override
    public boolean isFinished() {
        return true;
    }
}