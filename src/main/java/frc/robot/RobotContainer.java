package frc.robot;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.Cannon;
import frc.robot.Subsystems.Drivetrain;

public class RobotContainer {

    private final CommandXboxController m_driverController = new CommandXboxController(0);

    private final Drivetrain m_drivetrain = new Drivetrain();
    private final Cannon m_cannon = new Cannon();
    
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
                    * 0.3);
          }, m_drivetrain));

          // return new InstantCommand( () -> { shoot(); }); --> command syntax

        
        //Shoot PSI Values
        m_driverController.povDown().onTrue(new InstantCommand(() -> {m_cannon.setShootPSI(20);}));
        m_driverController.povLeft().onTrue(new InstantCommand(() -> {m_cannon.setShootPSI(30);}));
        m_driverController.povUp().onTrue(new InstantCommand(() -> {m_cannon.setShootPSI(40);}));
        m_driverController.povRight().onTrue(new InstantCommand(() -> {m_cannon.setShootPSI(50);}));

         m_driverController.x().onTrue(new RunCommand(() -> { m_cannon.goToPSI(); }, m_cannon).until(()->m_cannon.doneFilling()).andThen(()->{m_cannon.resetfiller();}, m_cannon));
         m_driverController.b().onTrue(new InstantCommand(() -> {m_cannon.shoot( m_driverController.getLeftTriggerAxis()); }, m_cannon).andThen(new WaitCommand(0.5)).andThen(()->{m_cannon.unshoot();}));
    }
}
