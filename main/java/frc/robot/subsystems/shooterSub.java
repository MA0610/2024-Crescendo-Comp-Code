// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class shooterSub extends SubsystemBase {
  /** Creates a new shooterSub. */
   TalonFX shooterFL, shooterBL, shooterFR, shooterBR, intake;

  public shooterSub() {
  shooterFL = new TalonFX(Constants.shooterFL);
  shooterBL = new TalonFX(Constants.shooterBL);
  shooterFR = new TalonFX(Constants.shooterFR);
  shooterBR = new TalonFX(Constants.shooterBR);               
  intake = new TalonFX(Constants.intakeMotor);

  shooterFL.setNeutralMode(NeutralModeValue.Coast);
  shooterBL.setNeutralMode(NeutralModeValue.Coast); //Should maybe be brake
  shooterFR.setNeutralMode(NeutralModeValue.Coast);
  shooterBR.setNeutralMode(NeutralModeValue.Coast); //Should maybe be brake

  intake.setInverted(true);
  shooterBR.setInverted(true);
  shooterFR.setInverted(true);
  shooterFL.setInverted(false);
  shooterBL.setInverted(false);




  }

  public void setPower(double power){
    shooterFL.set(power);
    shooterBL.set(power);
    shooterFR.set(power);
    shooterBR.set(power);
    }

  public void intake(){
    intake.set(0.15);
  }

  public void Shoot(){
    shooterFL.set(1.00);
    shooterFR.set(1.00);
    shooterBL.set(1.00);
    shooterBR.set(1.00);
    intake.set(0.80);
  }

  public void shooterSuck(){
    shooterFL.set(-0.5);
    shooterBL.set(-0.5);
    shooterFR.set(-0.5);
    shooterBR.set(-0.5);  
  }
  
  public void shooterSpit(){
    shooterFL.set(0.5);
    shooterBL.set(0.5);
    shooterFR.set(0.5);
    shooterBR.set(0.5);  
  }

  public void shooterStop(){
    shooterFL.set(0);
    shooterBL.set(0);
    shooterFR.set(0);
    shooterBR.set(0);  
    intake.set(0);
  }
  
  public void intakeStop(){
    intake.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
