// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Rumble;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Rumble.Rumble;

public class RumbleForTime extends Command {
  private Timer timer = new Timer();
  private double time;
  private double strength;
  private RumbleType type;

  private Rumble rumble = new Rumble();
  /** Creates a new RumbleForTime. */
  public RumbleForTime(RumbleType type, double strength, double time) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.type = type;
    this.strength = strength;
    this.time = time;
    addRequirements(rumble);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    rumble.setVibration(strength, type);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    rumble.setVibration(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() > time;
  }
}
