package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import java.util.function.Supplier;

public class SwerveDriveCommand extends Command {

  public final DriveSubsystem swerveSubsystem;
  public final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
  public Supplier<Boolean> fastModeFunction, fasterModeFunction;
  public final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
  public boolean fieldOrientedFunction, isPresetTurnActive;
  private ShuffleboardTab m_tab;
  private GenericEntry shuffleFieldOriented;

  public SwerveDriveCommand(
    DriveSubsystem subsystem,
    Supplier<Double> xSupplier,
    Supplier<Double> ySupplier,
    Supplier<Double> turnSupplier,
    boolean fieldOriented
  ){
    swerveSubsystem = subsystem;
    xSpdFunction = xSupplier;
    ySpdFunction = ySupplier;
    turningSpdFunction = turnSupplier;
    fieldOrientedFunction = fieldOriented;
  
    xLimiter = new SlewRateLimiter(DriveConstants.kXSlewRateLimit);
    yLimiter = new SlewRateLimiter(DriveConstants.kYSlewRateLimit);
    turningLimiter = new SlewRateLimiter(DriveConstants.kTurnSlewRateLimit);
    addRequirements(swerveSubsystem);

    m_tab = Shuffleboard.getTab("Swerve Instance");
    shuffleFieldOriented = m_tab.add("Field Oriented" + this.toString(), fieldOriented).withWidget(BuiltInWidgets.kToggleSwitch).getEntry();
  }

  @Override
  public void execute() {

    // Get inputs
    double xSpeed = 0, ySpeed = 0, turningSpeed = 0;
    boolean fastMode = false, fasterMode = false;
      xSpeed = xSpdFunction.get();
      ySpeed = ySpdFunction.get();
      turningSpeed = turningSpdFunction.get();
     
    
    
    // Death
    xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0;
    ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0;
    turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0;

    // Slew soup

    xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kPhysicalMaxSpeedMetersPerSecond;
    ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kPhysicalMaxSpeedMetersPerSecond;
    turningSpeed = turningLimiter.calculate(turningSpeed) * DriveConstants.maxTurnSpeed;

    // I am speed
    ChassisSpeeds chassisSpeeds;
    if (shuffleFieldOriented.getBoolean(fieldOrientedFunction)) {
      // Field oriented
      chassisSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
          xSpeed,
          ySpeed,
          turningSpeed,
          swerveSubsystem.getRotation2d()
        );
    } else {
      chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
    }

    // Divide and conker
    SwerveModuleState[] moduleStates =
      DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

    // Actually do the thing
    swerveSubsystem.setModuleStates(moduleStates);
  }

  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.resetEncoders();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}