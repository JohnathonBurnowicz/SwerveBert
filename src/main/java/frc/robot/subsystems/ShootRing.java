// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

/* 
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
/**
 * This command allows the turret to spin at a certain
 * speed for a certain time.
 * 
 * This is only used in autonomous.
 *//* 
public class ShootRing extends Command {
  //private final Shooter shooter;
    private final double speed;

    private int counter = 0;
    private int target = 0;
  /** Creates a new TurnTurretForTime. */
  /*public ShootRing(Shooter shooter, double speed, double seconds) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
        this.speed = speed;

        target = (int)(seconds * 50);

        addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {if(counter < target)
    counter++;

shooter.setRotatorSpeed(speed);}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {shooter.setRotatorSpeed(0);}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return counter >= target;
  }
}*/