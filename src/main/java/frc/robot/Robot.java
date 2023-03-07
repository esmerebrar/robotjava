package frc.robot;
package edu.wpi.first.wpilibj.examples.encoder;
package edu.wpi.first.wpilibj.examples.AnalogGyro;


import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smarDashboard;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.AnalogInput;

 
  public class Robot extends TimedRobot {
  private final Timer m_timer = new Timer();
  private AnalogInput kGyroPort; 
  private DifferentialDrive m_myRobot;
  private final CANSparkMax m_leftDrive = new CANSparkMax(1,MotorType.kBrushless);
  private final CANSparkMax y_leftDrive = new CANSparkMax(2,MotorType.kBrushless);
  private final CANSparkMax y_rightDrive = new CANSparkMax(3,MotorType.kBrushless);
  private final CANSparkMax m_rightDrive= new CANSparkMax(4,MotorType.kBrushless);
  private final Joystick m_stick = new Joystick(0);
  private final Encoder m_encoder = new Encoder (1,2 false,CounterBase.EncodingType.k4X);
  private static final double kAngleSetpoint = 0.0;
  private static final double kP = 0.005;
  private static final double kVoltsPerSecond = 0.0128;
  private static final int kLeftMotorPort = 0;
  private static final int tLeftMotorPort = 1;
  private static final int kRightMotorPort = 2;
  private static final int tRightMotorPort = 3;
  private static final int kJoystickPort = 0;
  private final AnalogGyro m_gyro = new AnalogGyro(kGyroPort);
  private final Joystick m_joystick = new Joystick(kJoystickPort);
 
  
  @Override
  public void robotInit() {

  m_encoder.setSamplesToAverage(5);
  m_encoder.setDistancePerPulse(1.0 /360.0*2.0* Math.PI * 1.5);
  m_encoder.setMinRate(1.0);
  m_gyro.setSensitivity(kVoltsPerSecond);
  ((MotorController) m_rightDrive).setInverted(true);

  }

  @Override
  public void teleopPeriodic() {
  SmartDashboard.putNumber("Encoder Distance", m_encoder.getDistance());
  SmartDashboard.putNumber("Encoder Rate", m_encoder.getRate());
  double turningValue = (kAngleSetpoint- m_gyro.getAngle()); * kP;
  m_myRobot.arcadeDrive(-m_joystick.getY(), -turningValue);
  
  }
 }




 
 /* 
  @Override
  public void autonomousInit() {
    m_timer.restart();
  }
  /* 
  @Override
  public void autonomousPeriodic() {
    DifferentialDrive m_robotDrive;
    if (m_timer.get() < 3.0) {
      m_robotDrive.arcadeDrive(0.5,0.0,false);
    } else {
      m_robotDrive.stopMotor();
    }
  }
  /* 
  @Override
  public void teleopPeriodic() {
    DifferentialDrive m_robotDriveCanSparkMax;
    m_robotDriveCanSparkMax.arcadeDrive( m_stick.getY(), m_stick.getX());
  }
}

  