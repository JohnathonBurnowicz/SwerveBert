// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.GlobalVariables;

public class ArmDown extends Command {
  
  
  private final Arm arm;
  private int armDown;
  /** Creates a new RotateArmToAngle. */
  public ArmDown(Arm arm) {
    this.arm = arm;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    armDown = GlobalVariables.downPosition;
    arm.setArmAnglePID(Constants.ARM_POSITIONS.get(armDown));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(arm.isArmInPosition(Constants.ARM_POSITIONS.get(armDown))) {
      return true;
    }else{
      return false;
    }
  }
}