package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RobotSuperstructure;

public class EmergencyStopCommand extends Command {
    private final RobotSuperstructure m_superstructure;
    
    public EmergencyStopCommand(RobotSuperstructure superstructure) {
        m_superstructure = superstructure;
        addRequirements(superstructure);
    }
    
    @Override
    public void initialize() {
        m_superstructure.emergencyStop();
    }
    
    @Override
    public boolean isFinished() {
        return true;
    }
}