// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  AnalogInput FLEncoder = new AnalogInput(0);
  AnalogInput FREncoder = new AnalogInput(1);
  AnalogInput BLEncoder = new AnalogInput( 2);
  AnalogInput BREncoder = new AnalogInput(3);

  PIDController PID1 = new PIDController(0, 0, 0);
  PIDController PID2 = new PIDController(0, 0, 0);
  PIDController PID3 = new PIDController(0, 0, 0);
  PIDController PID4 = new PIDController(0, 0, 0);

  CANSparkMax motor1 = new CANSparkMax(0, MotorType.kBrushless);
  CANSparkMax motor2 = new CANSparkMax(0, MotorType.kBrushless);
  CANSparkMax motor3 = new CANSparkMax(0, MotorType.kBrushless);
  CANSparkMax motor4 = new CANSparkMax(0, MotorType.kBrushless);

  XboxController control = new XboxController(0);

  double OffSet1 = 0.0; 
  double OffSet2 = 0.0; 
  double OffSet3 = 0.0; 
  double OffSet4 = 0.0; 
    


  public double getAngle(AnalogInput Encoder , double Offset){
     
    double Bits = Encoder.getValue();
    double angle = (Bits*360/4096);

    return angle;
  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    PID1.enableContinuousInput(-180, 180);
    PID2.enableContinuousInput(-180, 180);
    PID3.enableContinuousInput(-180, 180);
    PID4.enableContinuousInput(-180, 180);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {

   
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {


  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() { 
    // Calcula la posición actual del encoder en grados
    boolean ButtonA = control.getAButton();
    boolean ButtonB = control.getBButton();
    boolean ButtonX = control.getXButton();
    boolean ButtonY = control.getYButton();

    if (ButtonA == true) {
      motor1.set(PID1.calculate(getAngle(FLEncoder, OffSet1), 90));
    }
    else {
      motor1.set(0);
    }

    if (ButtonB == true) {
      motor2.set(PID2.calculate(getAngle(FREncoder, OffSet2), 90));
    }
    else {
      motor2.set(0);
    }
    
    if (ButtonX == true) {
      motor3.set(PID3.calculate(getAngle(BLEncoder, OffSet3), 90));
    }
    else {
      motor3.set(0);
    }

    if (ButtonY == true) {
      motor4.set(PID4.calculate(getAngle(BREncoder, OffSet4), 90));
    }
    else {
      motor4.set(0);
    }

  SmartDashboard.putNumber("Angulo FL", getAngle(FLEncoder, OffSet1));
  SmartDashboard.putNumber("Angulo FR", getAngle(FREncoder, OffSet2));  
  SmartDashboard.putNumber("Angulo BL", getAngle(BLEncoder, OffSet3));
  SmartDashboard.putNumber("Angulo BR", getAngle(BREncoder, OffSet4));
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
