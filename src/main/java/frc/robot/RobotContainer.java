package frc.robot;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.Drivetrain;

public class RobotContainer {

    private final CommandXboxController m_driverController = new CommandXboxController(0);

    private final Drivetrain m_drivetrain = new Drivetrain();
    
    public RobotContainer(){

        m_drivetrain.setDefaultCommand(Commands.run(() -> {
            double Smodifier = m_driverController.getRightTriggerAxis() * 0.5 + 0.5
                - m_driverController.getLeftTriggerAxis() * 0.42;
            // double Tmodifer =1-(( m_driverController.leftBumper().getAsBoolean()?0:1 )*
            // (m_driverController.getRightTriggerAxis()>0.5 &&
            // m_driverController.getLeftTriggerAxis()<0.5?1:0)*0.4); //
            double Tmodifier =
                (Math.abs(m_driverController.getLeftY()) < 0.25 ? Smodifier : Math.min(0.5, Smodifier));
      
            m_drivetrain.smoothDrive(
                -1 * m_driverController.getLeftY() * Math.abs(m_driverController.getLeftY()) * Smodifier
                    * 1,
                m_driverController.getRightX() * -1 * Math.abs(m_driverController.getRightX()) * Tmodifier
                    * 1);
          }, m_drivetrain));

    }
}
