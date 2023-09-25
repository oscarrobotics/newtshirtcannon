package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase{
    
    //Where we establish our motors as objects on the robot
    private final CANSparkMax m_leftMaster = new CANSparkMax(3, MotorType.kBrushless);
    private final CANSparkMax m_leftDrone = new CANSparkMax(4, MotorType.kBrushless);
    private final CANSparkMax m_rightMaster = new CANSparkMax(1, MotorType.kBrushless);
    private final CANSparkMax m_rightDrone = new CANSparkMax(2, MotorType.kBrushless);
    
    //gyro
    private final WPI_Pigeon2 m_gyro = new WPI_Pigeon2(0);

    /*This is where motor groups are established. 
        Motor groups are essential for Differential Drive (what we're using for this particular drivetrain),
        given that they combine two separate motors into one controllable object.
    */
    private final MotorControllerGroup m_leftMotors = new MotorControllerGroup(m_leftMaster, m_leftDrone);
    private final MotorControllerGroup m_rightMotors = new MotorControllerGroup(m_rightMaster, m_rightDrone);


    //PID Controller obj
    private final SparkMaxPIDController m_leftPIDController;
    private final SparkMaxPIDController m_rightPIDController;
    //ratio PID controller

    //Encoder obj
    // private final SparkMaxRelativeEncoder m_leftEncoder = m_leftMaster.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);


    /* Differential drivetrain kinematics determine the velocities 
   to be calculated for each side of the differential drive train. 
   
   To use this calculation, the
   TRACK WIDTH (space between the left/right sides of the wheels in meters) must be placed as a 
   parameter. */

    public final DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(25));

    /* */
    public final DifferentialDrivePoseEstimator m_poseEstimator = 
        new DifferentialDrivePoseEstimator(
                m_kinematics, new Rotation2d(0), 0.0, 0.0, new Pose2d());


    //worry about vision later


    final int kCountsPerRev = 1; //How many times the encoder will count per one revolution of the motor.
    final double kGearRatio = 10.71; //The gear ratio of the encoder relative to the wheel diameter.
    final double kWheelDiameterInches = 3.55; //Radius of the wheel (inches)
    final int k100msPerSecond = 10;
    
    final double maxspeed = 1;

    final double kP = 0.0002;
    final double kI = 0;
    final double kD = 0;
    final double kIz = 0;
    final double kFF = 0.0002;

    private SlewRateLimiter slewLimit = new SlewRateLimiter(6, -6, 0);

    public Drivetrain(){

        m_gyro.reset();

        m_rightMaster.setInverted(true);
        m_rightDrone.setInverted(true);
        m_leftMaster.setInverted(false);
        m_leftDrone.setInverted(false);

        m_leftMaster.setIdleMode(IdleMode.kBrake);
        m_leftDrone.setIdleMode(IdleMode.kBrake);
        m_rightMaster.setIdleMode(IdleMode.kBrake);
        m_rightDrone.setIdleMode(IdleMode.kBrake);

        //config PID values
        m_leftPIDController = m_leftMaster.getPIDController();
        m_rightPIDController = m_rightMaster.getPIDController();
        
        m_leftPIDController.setP(kP);
        m_leftPIDController.setI(kI);
        m_leftPIDController.setD(kD);
        m_leftPIDController.setIZone(kIz);
        m_leftPIDController.setFF(kFF);

        m_rightPIDController.setP(kP);
        m_rightPIDController.setI(kI);
        m_rightPIDController.setD(kD);
        m_rightPIDController.setIZone(kIz);
        m_rightPIDController.setFF(kFF);


    }

    public void smoothDrive(double speed, double rotation){
    
        //filter coefficient for the low pass filter high value means more smoothing
         
        //apply a low pass filter to the speed input
        //takes a percentage of the new speed and the oposite percentage of the old speed and adds them together
        // speed = speed * (1-kFiltercoeff) + filtspeed * kFiltercoeff ;
        speed = slewLimit.calculate(speed);
        //use the differntial drive invers kinematics class to set the motor controllers directly in velocity control mode
        DifferentialDriveWheelSpeeds wheelSpeeds = m_kinematics.toWheelSpeeds(new ChassisSpeeds(speed, 0, rotation));
        
        System.out.print(wheelSpeeds.leftMetersPerSecond);
        System.out.print("*");
        System.out.println(wheelSpeeds.rightMetersPerSecond);
       
        //Convert m/s to RPM
        double leftspeedtalon = VelocitytoRPM( wheelSpeeds.leftMetersPerSecond );
        double rightspeedtalon = VelocitytoRPM( wheelSpeeds.rightMetersPerSecond );
        System.out.print(leftspeedtalon);
        System.out.print("  ");
        System.out.println(rightspeedtalon);

        System.out.print(m_leftMaster.getEncoder().getVelocity());
        System.out.print("  ");
        System.out.println(m_rightMaster.getEncoder().getVelocity());
    
        m_leftPIDController.setReference(leftspeedtalon,CANSparkMax.ControlType.kVelocity);
        m_rightPIDController.setReference(rightspeedtalon,CANSparkMax.ControlType.kVelocity);
        m_leftDrone.follow(m_leftMaster);
        m_rightDrone.follow(m_rightMaster);
      }



    // private int velocityToNativeUnits(double velocityMetersPerSecond){
    //     double wheelRotationsPerSecond = velocityMetersPerSecond/(2 * Math.PI * Units.inchesToMeters(kWheelRadiusInches));
    //     double motorRotationsPerSecond = wheelRotationsPerSecond * kGearRatio;
    //     double motorRotationsPer100ms = motorRotationsPerSecond / k100msPerSecond;
    //     int sensorCountsPer100ms = (int)(motorRotationsPer100ms * kCountsPerRev);
    //     return sensorCountsPer100ms;
    //   }


  
    
      //m/s to rpm
      //Velocity(m/s) / wheel circumference (meters/revolution) * gear ratio (unitless) * 60 (seconds) --> dimensional analysis
      private double VelocitytoRPM(double velocityMetersPerSecond){

       return (velocityMetersPerSecond * 60 * kGearRatio) / ((Units.inchesToMeters(kWheelDiameterInches) * Math.PI));

      }

}