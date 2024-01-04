// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SuperStructure extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public SuperStructure() {}

  // ==================== OBJECTS ====================

  private static SuperStructure m_instance = new SuperStructure();

  public Wrist m_wrist = Wrist.getInstance();

  // ==================== ENUMS ====================

  enum SuperStructureStates() {
    HIGH,
    MEDIUM,
    LOW,

    STOW
  } 

  private SuperStructureStates state = SuperStructureStates.STOW;



  // ==================== METHODS ====================

  public static SuperStructure getInstance() {
    return m_instance;
  }

  public void setState(SuperStructureStates state) {
    this.state = state;

    switch(state){
      case HIGH:
        goToSetPoint(Rotation2d.fromDegrees(100));
        break;
      case MEDIUM:
        break;
      case LOW:
        break;
      case STOW:
        m_wrist.setState(WristStates.OFF);
        break;
    }

  }

  public void goToSetPoint(Rotation2d angle){
    m_wrist.setSetPoint(angle);
  }


}
