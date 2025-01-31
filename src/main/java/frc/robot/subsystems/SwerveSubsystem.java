// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.File;
import java.util.function.DoubleSupplier;
import java.util.Optional;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.robot.Constants;
import frc.robot.subsystems.Vision.Cameras;
import org.photonvision.targeting.PhotonTrackedTarget;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.SwerveDriveTest;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import org.photonvision.targeting.PhotonPipelineResult;


public class SwerveSubsystem extends SubsystemBase {
  /** Swerve drive object */
  private final SwerveDrive  swerveDrive;
  /** AprilTag field layout */
  private final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
  /** Enable vision odometry updates while driving */
  private final boolean visionBasedOdometry = true;
  /** PhotonVision class to keep an accurate odometry */
  private final Vision vision;

  // Load the RobotConfig from the GUI settings.
  private final RobotConfig PPconfig;

  /**
   * Creates a new SwerveSubsystem.
   */
  public SwerveSubsystem() {
    // Enable high telemetry (allows to see more data in ShuffleBoard)
    SwerveDriveTelemetry.verbosity=TelemetryVerbosity.HIGH;

    // Initialize swerve drive
    try {
      swerveDrive = new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve")).createSwerveDrive(Constants.MAX_SPEED);
    } catch (Exception e) {
      throw new RuntimeException(e);
    }

    // Keep the robot heading the same as the previous heading while the robot is translating.
    swerveDrive.setHeadingCorrection(false);

    // Reduces unintended skew in a robot's movement by adjusting angular velocity
    // TODO Tune it by moving the robot in a straight line while rotating and adjusting the compensation coefficient until the deviation is minimized.
    swerveDrive.setAngularVelocityCompensation(false, false, 0.1);

    // Synchronize the absolute encoders and relative encoders everytime bot is at rest for half-a-second
    swerveDrive.setModuleEncoderAutoSynchronize(true, 1);

    // TODO What exactly does this do!?
    swerveDrive.pushOffsetsToEncoders();

    // Set motors to brake mode
    setMotorBrake(true);

    if (visionBasedOdometry)
    {
      // Intialize PhotonVision
      vision = new Vision(swerveDrive::getPose, swerveDrive.field);

      // Stop the odometry thread if we are using vision that way we can synchronize updates better.
      swerveDrive.stopOdometryThread();
    }

    try
    {
      // Load PathPlanner config
      PPconfig = RobotConfig.fromGUISettings();

      // Configure AutoBuilder (for PathPlanner)
      AutoBuilder.configure(
        this::getPose, // Robot pose supplier
        this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getRobotVelocity, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        (speeds, feedforwards) -> swerveDrive.drive(speeds, swerveDrive.kinematics.toSwerveModuleStates(speeds), feedforwards.linearForces()), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
        new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
          // Translation PID constants
          new PIDConstants(
            swerveDrive.swerveDriveConfiguration.modules[0].getDrivePIDF().p,
            swerveDrive.swerveDriveConfiguration.modules[0].getDrivePIDF().i,
            swerveDrive.swerveDriveConfiguration.modules[0].getDrivePIDF().d
          ),
          // Rotation PID constants
          new PIDConstants(
            swerveDrive.swerveDriveConfiguration.modules[0].getAnglePIDF().p,
            swerveDrive.swerveDriveConfiguration.modules[0].getAnglePIDF().i,
            swerveDrive.swerveDriveConfiguration.modules[0].getAnglePIDF().d
          )
        ),
        PPconfig, // The robot configuration
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent())
          {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this // Reference to this subsystem to set requirements
      );
    } catch (Exception e)
    {
      throw new RuntimeException(e);
    }
  }


  @Override
  public void periodic() {
    // When vision is enabled we must manually update odometry in SwerveDrive
    if (visionBasedOdometry)
    {
      swerveDrive.updateOdometry();
      vision.updatePoseEstimation(swerveDrive);
    }

    try {
      List<PhotonPipelineResult> pipelineResults = Cameras.CAM1.camera.getAllUnreadResults();
      if(!pipelineResults.isEmpty()) {
        PhotonPipelineResult LatestResult = pipelineResults.get(pipelineResults.size() - 1);
        PhotonTrackedTarget LatestBestTarget = LatestResult.getBestTarget();
        Shuffleboard.getTab("April Tag ID").add("Latest Result", String.valueOf(LatestBestTarget.getFiducialId()), "-1");
        Shuffleboard.getTab("Area of Tag").add("Latest Result", String.valueOf(LatestBestTarget.getArea()));
        Shuffleboard.getTab("Photon Yaw").add("Latest Result", String.valueOf(LatestBestTarget.getYaw()));
        Shuffleboard.getTab("Photon Pitch").add("Latest Result", String.valueOf(LatestBestTarget.getPitch()));
      }else{
      }
    } catch (Exception e) {
      e.printStackTrace();
    }

  }


  /**
   * Command to drive the robot using translative values and heading as angular velocity.
   *
   * @param translationX     Translation in the X direction (forward).
   * @param translationY     Translation in the Y direction (left).
   * @param angularRotationX Rotation of the robot to set
   * @param openLoop         If true will use open-loop velocity(higher responsiveness - better for auton), else will use close-loop velocity(higher precision - better for teleop).
   * @return Drive command.
   */
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX, boolean fieldRelative, boolean openLoop)
  {
    return run(() -> {
      // Make the robot drive
      swerveDrive.drive(
        new Translation2d(
          translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
          translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity()
        ),
        angularRotationX.getAsDouble() * swerveDrive.getMaximumChassisAngularVelocity(),
        fieldRelative,
        openLoop);
    });
  }


  /**
   * Drive according to the chassis robot oriented velocity.
   *
   * @param velocity Robot oriented {@link ChassisSpeeds}
   */
  public void drive(ChassisSpeeds velocity)
  {
    swerveDrive.drive(velocity);
  }


  /**
   * Get the swerve drive kinematics object.
   *
   * @return {@link SwerveDriveKinematics} of the swerve drive.
   */
  public SwerveDriveKinematics getKinematics()
  {
    return swerveDrive.kinematics;
  }


  /**
   * Resets odometry to the given pose. Gyro angle and module positions do not need to be reset when calling this
   * method. However, if either gyro angle or module position is reset, this must be called in order for odometry to
   * keep working.
   *
   * @param initialHolonomicPose The pose to set the odometry to
   */
  public void resetOdometry(Pose2d initialHolonomicPose)
  {
    swerveDrive.resetOdometry(initialHolonomicPose);
  }


  /**
   * Gets the current pose (position and rotation) of the robot, as reported by odometry.
   *
   * @return The robot's pose
   */
  public Pose2d getPose()
  {
    return swerveDrive.getPose();
  }


  /**
   * Set chassis speeds with closed-loop velocity control.
   *
   * @param chassisSpeeds Chassis Speeds to set.
   */
  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds)
  {
    swerveDrive.setChassisSpeeds(chassisSpeeds);
  }


  /**
   * Post the trajectory to the field.
   *
   * @param trajectory The trajectory to post.
   */
  public void postTrajectory(Trajectory trajectory)
  {
    swerveDrive.postTrajectory(trajectory);
  }


  /**
   * Resets the gyro angle to zero and resets odometry to the same position, but facing toward 0.
   */
  public void zeroGyro()
  {
    swerveDrive.zeroGyro();
  }


  /**
   * Checks if the alliance is red, defaults to false if alliance isn't available.
   *
   * @return true if the red alliance, false if blue. Defaults to false if none is available.
   */
  private boolean isRedAlliance()
  {
    var alliance = DriverStation.getAlliance();
    return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
  }


  /**
   * This will zero (calibrate) the robot to assume the current position is facing forward
   *
   * If red alliance rotate the robot 180 after the drivebase zero command
   */
  public void zeroGyroWithAlliance()
  {
    if (isRedAlliance())
    {
      zeroGyro();
      //Set the pose 180 degrees
      resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(180)));
    } else
    {
      zeroGyro();
    }
  }


  /**
   * Sets the drive motors to brake/coast mode.
   *
   * @param brake True to set motors to brake mode, false for coast.
   */
  public void setMotorBrake(boolean brake)
  {
    swerveDrive.setMotorIdleMode(brake);
  }


  /**
   * Gets the current yaw angle of the robot, as reported by the swerve pose estimator in the underlying drivebase.
   * Note, this is not the raw gyro reading, this may be corrected from calls to resetOdometry().
   *
   * @return The yaw angle
   */
  public Rotation2d getHeading()
  {
    return getPose().getRotation();
  }


  /**
   * Get the chassis speeds based on controller input of 2 joysticks. One for speeds in which direction. The other for
   * the angle of the robot.
   *
   * @param xInput   X joystick input for the robot to move in the X direction.
   * @param yInput   Y joystick input for the robot to move in the Y direction.
   * @param headingX X joystick which controls the angle of the robot.
   * @param headingY Y joystick which controls the angle of the robot.
   * @return {@link ChassisSpeeds} which can be sent to the Swerve Drive.
   */
  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX, double headingY)
  {
    Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));
    return swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(), scaledInputs.getY(), headingX, headingY, getHeading().getRadians(), Constants.MAX_SPEED);
  }


  /**
   * Get the chassis speeds based on controller input of 1 joystick and one angle. Control the robot at an offset of
   * 90deg.
   *
   * @param xInput X joystick input for the robot to move in the X direction.
   * @param yInput Y joystick input for the robot to move in the Y direction.
   * @param angle  The angle in as a {@link Rotation2d}.
   * @return {@link ChassisSpeeds} which can be sent to the Swerve Drive.
   */
  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle)
  {
    Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));

    return swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(), scaledInputs.getY(), angle.getRadians(), getHeading().getRadians(), Constants.MAX_SPEED);
  }


  /**
   * Gets the current field-relative velocity (x, y and omega) of the robot
   *
   * @return A ChassisSpeeds object of the current field-relative velocity
   */
  public ChassisSpeeds getFieldVelocity()
  {
    return swerveDrive.getFieldVelocity();
  }


  /**
   * Gets the current velocity (x, y and omega) of the robot
   *
   * @return A {@link ChassisSpeeds} object of the current velocity
   */
  public ChassisSpeeds getRobotVelocity()
  {
    return swerveDrive.getRobotVelocity();
  }


  /**
   * Get the {@link SwerveController} in the swerve drive.
   *
   * @return {@link SwerveController} from the {@link SwerveDrive}.
   */
  public SwerveController getSwerveController()
  {
    return swerveDrive.swerveController;
  }


  /**
   * Get the {@link SwerveDriveConfiguration} object.
   *
   * @return The {@link SwerveDriveConfiguration} fpr the current drive.
   */
  public SwerveDriveConfiguration getSwerveDriveConfiguration()
  {
    return swerveDrive.swerveDriveConfiguration;
  }


  /**
   * Lock the swerve drive to prevent it from moving.
   */
  public void lock()
  {
    swerveDrive.lockPose();
  }


  /**
   * Add a fake vision reading for testing purposes.
   */
  public void addFakeVisionReading()
  {
    swerveDrive.addVisionMeasurement(new Pose2d(3, 3, Rotation2d.fromDegrees(65)), Timer.getFPGATimestamp());
  }


  /**
   * Gets the swerve drive object.
   *
   * @return {@link SwerveDrive}
   */
  public SwerveDrive getSwerveDrive()
  {
    return swerveDrive;
  }


  /**
   * Command to characterize the robot drive motors using SysId
   *
   * @return SysId Drive Command
   */
  public Command sysIdDriveMotorCommand()
  {
    return SwerveDriveTest.generateSysIdCommand(
        SwerveDriveTest.setDriveSysIdRoutine(
            new Config(),
            this, swerveDrive, 12, true),
        3.0, 5.0, 3.0);
  }

  /**
   * Command to characterize the robot angle motors using SysId
   *
   * @return SysId Angle Command
   */
  public Command sysIdAngleMotorCommand()
  {
    return SwerveDriveTest.generateSysIdCommand(
        SwerveDriveTest.setAngleSysIdRoutine(
            new Config(),
            this, swerveDrive),
        3.0, 5.0, 3.0);
  }


  /**
   * Aim the robot at the target returned by PhotonVision.
   *
   * @return A {@link Command} which will run the alignment.
   */
  public Command aimAtTarget(Cameras camera)
  {

    return run(() -> {

      Optional<PhotonPipelineResult> resultO = camera.getBestResult();
      if (resultO.isPresent())
      {
        var result = resultO.get();
        if (result.hasTargets())
        {
          drive(getTargetSpeeds(0, 0, Rotation2d.fromDegrees(result.getBestTarget().getYaw()))); // Not sure if this will work, more math may be required.
        }
      }
    });
  }
}
