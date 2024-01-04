// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.Constants.WristConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;





public class Wrist extends SubsystemBase {

  // ==================== STATES ====================

  enum WristStates {
    OFF,
    JOG,
    POSITION,
    ZERO
  }

  double kJogValue = 0;
  Rotation2d m_setPoint = new Rotation2d();

  WristStates m_state = WristStates.OFF;

  // ==================== OBJECTS ====================

  CANSparkMax m_leftMotor = new CANSparkMax(WristConstants.kLeftMotor, MotorType.kBrushless);
  CANSparkMax m_rightMotor = new CANSparkMax(WristConstants.kRightMotor, MotorType.kBrushless);

  AbsoluteEncoder m_encoder = m_rightMotor.getAbsoluteEncoder(Type.kDutyCycle);

  DigitalInput m_limitSwitch = new DigitalInput(0);

  static Wrist m_instance = new Wrist();

  SparkMaxPIDController m_PIDController = m_rightMotor.getPIDController();


  // ==================== METHODS ====================
  
  // GETTERS

  public static Wrist getInstance() {
    return m_instance;
  }

  public double getJogValue() {
    return kJogValue;
  }

  public Rotation2d getSetPoint() {
    return m_setPoint;
  }

  public WristStates getState() {
    return m_state;
  }

  // SETTERS

  private void setJogValue(double jogValue){
    kJogValue = jogValue;
  }

  private void setSetPoint(Rotation2d setPoint){
    setState(WristStates.POSITION);
    m_setPoint = setPoint;
  }

  private void setState(WristStates state){
    m_state = state;
  }

  // MISC

  public void goToPosition() {
    m_PIDController.setReference(m_setPoint.getRotations() * WristConstants.kWRIST_GEAR_RATIO, ControlType.kSmartMotion);
  }

  private void zero() {
    this.m_jogValue = m_limitSwitch.get() ? 0.0 : 0.1;
  }

  private void OFF() {
    set(0);
  }

  public void set(double value) {
    m_rightMotor.set(value);
  }

  // ==================== SUBSYSTEM ===================

  public CommandBase exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    switch(WristStates) {
      case OFF:
        OFF();
        break;
      case JOG:
        set(kJogValue);
        break;
      case POSITION:
        goToPosition();
        break;
      case ZERO:
        zero();
        break;
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  // ==================== MOTORS ====================

  public Wrist() {
    configMotors();
  }


  public void configMotors() {
    m_leftMotor.follow(m_rightMotor);
    m_leftMotor.setInverted(true);
    
    m_leftMotor.restoreFactoryDefaults();
    m_rightMotor.restoreFactoryDefaults();

    m_leftMotor.setIdleMode(IdleMode.kBrake);
    m_rightMotor.setIdleMode(IdleMode.kBrake);

    m_leftMotor.setSmartCurrentLimit(35);
    m_rightMotor.setSmartCurrentLimit(35);

    m_PIDController.setFF(WristConstants.kFF);
    m_PIDController.setP(WristConstants.kP);
    m_PIDController.setD(WristConstants.kD);
  }
}
