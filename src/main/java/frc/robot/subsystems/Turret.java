// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


import frc.robot.Constants.*;
import frc.robot.Constants;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Turret extends SubsystemBase {
  /** Creates a new Shooter. */

  private final CANSparkMax left = new CANSparkMax(Constants.SHOOTER_LEFT, MotorType.kBrushless);
  private final CANSparkMax right = new CANSparkMax(Constants.SHOOTER_RIGHT, MotorType.kBrushless);

  private final SparkMaxPIDController pid = right.getPIDController();
  private final RelativeEncoder encoder = right.getEncoder();

  // Variables to keep track of RPM history over last 400ms.
  // Used to determine shooter stability.
  private double[] rpmLog = new double[10];//April 6 change - was 20 for the length
  private int rpmLogCounter = 0;

  private boolean gateOpen;

  private double targetRPM;
  private double rpmOffset;

  public Turret() {
//    System.out.println("Shooter constructor called");
    left.restoreFactoryDefaults();
    right.restoreFactoryDefaults();

    right.setOpenLoopRampRate(1);
    right.setClosedLoopRampRate(1);

    pid.setFeedbackDevice(encoder);
    pid.setFF(Constants.SHOOTER_FF);
    pid.setP(Constants.SHOOTER_P);
    
    right.setInverted(true);        
    left.follow(right, true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Keep track of previous 400ms of RPMs.
    // (not in chronological order)
    rpmLog[rpmLogCounter] = getFlywheelRPM();

    if(rpmLogCounter < rpmLog.length - 1)
        rpmLogCounter++;
    else
        rpmLogCounter = 0;

    SmartDashboard.putNumber("flywheelRPM", encoder.getVelocity());
    SmartDashboard.putNumber("rpmOffset", rpmOffset);
    }

    /**
     * Determines whether or not the shooter has been
     * stable for the past 400ms.
     * "Stable" = difference between highest and lowest RPM
     * is within acceptable range.
     * 
     * Ignores RPMs that are zero and datasets that are 50% zero.
     * 
     * Used to decide whether or not to shoot a ball.
     * @return if the shooter is stable
     */
  public boolean isStable(){
      double highest = Double.MIN_VALUE;
      double lowest = Double.MAX_VALUE;
      double rpmStabilityError = Constants.RPM_STABILITY_ERROR;
      double sum = 0;
      double nonZero = 0;
        for (int i = 0; i < rpmLog.length; i++){
          double thisRPM = rpmLog[i];
          if(thisRPM == 0)
              continue;

          nonZero++;
        if(thisRPM < lowest)
            lowest = thisRPM;
        if(thisRPM > highest)
            highest = thisRPM;
            sum += thisRPM;
        }

        if(nonZero < 10)
            return false;

        double average = sum / nonZero; 
    
      return (highest - average) < Constants.RPM_STABILITY_ERROR && (average - lowest) < Constants.RPM_STABILITY_ERROR;
  }

  /**
   * Sets flywheel's target RPM.
   * Adds an "RPM offset" that can be changed
   * on-the-fly by the operator to compensate for
   * battery voltage or power cell condition.
   * @param rpm
   */
  public void setFlywheelRPM(double rpm){
      targetRPM = rpm;
      pid.setReference(targetRPM + rpmOffset, CANSparkMax.ControlType.kVelocity);
  }

  public void setFlywheelPercent(double percent){
      right.set(percent);
  }

  public void stopMotors(){
      right.set(0);
  }

  public double getFlywheelRPM(){
      return encoder.getVelocity();
  }
  public void incrementRPMOffset(double amount){
      rpmOffset += amount;
      setFlywheelRPM(targetRPM);
      //Logger.info("Incremented RPM offset to " + rpmOffset);
  }

  public void decrementRPMOffset(double amount){
      rpmOffset -= amount;
      setFlywheelRPM(targetRPM);
      //Logger.info("Decremented RPM offset to " + rpmOffset);
  }
}