// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import oi.limelightvision.limelight.frc.LimeLight;
import oi.limelightvision.limelight.frc.ControlMode.CamMode;
import oi.limelightvision.limelight.frc.ControlMode.LedMode;

public class DriveWithLimeLightStearing extends CommandBase {
  
  private final Joystick m_leftJoystick;
  private final LimeLight m_LimeLight;
  private final Drivetrain m_DriveTrain;
  
  /** Creates a new DriveWithLimeLightStearing. */
  public DriveWithLimeLightStearing(Joystick leftstick, LimeLight limeLight, Drivetrain drivetrain) {
    m_leftJoystick = leftstick;
    m_LimeLight = limeLight;
    m_DriveTrain = drivetrain;

    addRequirements(m_DriveTrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //set up the limelight
    //use pipeline 1 for vision
    int pipeline = 1;
    m_LimeLight.setPipeline(pipeline);
    //m_LimeLight.setLEDMode(LedMode.kforceOn);
    //m_LimeLight.setCamMode(CamMode.kvision);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /**
     * kp is a proportional multiplier applied to the turn command of the drivetrain in this case
     * with kp = to 0.1 if the target is 10 deg off then a 1.0 will be command to the turn. This should be
     * tuned to make the drivetrain responcive but not to hot and occilations.
    */

    double kp = 0.1;
    double leftSpeed = m_leftJoystick.getRawAxis(1);
    double rightSpeed = 0 ;
    //Be sure a Target is found
    if(m_LimeLight.getIsTargetFound()){
      rightSpeed = m_LimeLight.getdegRotationToTarget() * kp;
    }else{
      rightSpeed = 0;
    }
    m_DriveTrain.my_DriveTankDrive(leftSpeed, rightSpeed);
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_DriveTrain.my_DriveTankDrive(0.0, 0.0);
    //Usig Pipeling Zero for driver mode LED are off
    int pipeline = 0;
    m_LimeLight.setPipeline(pipeline);
    //m_LimeLight.setLEDMode(LedMode.kforceOn);
    //m_LimeLight.setCamMode(CamMode.kvision);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
