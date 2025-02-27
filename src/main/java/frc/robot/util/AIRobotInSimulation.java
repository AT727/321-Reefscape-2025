/* (C) Robolancers 2025 */
package frc.robot.util;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.subsystems.drivetrain.DrivetrainConstants;
import java.util.Arrays;
import java.util.function.Supplier;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SelfControlledSwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.utils.FieldMirroringUtils;

public class AIRobotInSimulation extends SubsystemBase {
  /* If an opponent robot is not on the field, it is placed in a queening position for performance. */
  public static final Pose2d[] ROBOT_QUEENING_POSITIONS =
      new Pose2d[] {
        new Pose2d(-6, 0, new Rotation2d()),
        new Pose2d(-5, 0, new Rotation2d()),
        new Pose2d(-4, 0, new Rotation2d()),
        new Pose2d(-3, 0, new Rotation2d()),
        new Pose2d(-2, 0, new Rotation2d())
      };

  public static final Pose2d[] ROBOTS_STARTING_POSITIONS =
      new Pose2d[] {
        new Pose2d(15, 6, Rotation2d.fromDegrees(180)),
        new Pose2d(15, 4, Rotation2d.fromDegrees(180)),
        new Pose2d(15, 2, Rotation2d.fromDegrees(180)),
        new Pose2d(1.6, 6, new Rotation2d()),
        new Pose2d(1.6, 4, new Rotation2d())
      };

  private final SelfControlledSwerveDriveSimulation driveSimulation;
  private final Pose2d startingPose;
  private final int id;
  public static final AIRobotInSimulation[] instances =
      new AIRobotInSimulation[1]; // you can create as many opponent robots as you needs
  public static StructArrayPublisher<Pose2d> opponentPosePublisher =
      NetworkTableInstance.getDefault()
          .getStructArrayTopic("OpponentPoses", Pose2d.struct)
          .publish();

  public AIRobotInSimulation(int id) {
    this.id = id;
    this.startingPose = ROBOTS_STARTING_POSITIONS[id];
    this.driveSimulation =
        new SelfControlledSwerveDriveSimulation(
            new SwerveDriveSimulation(DrivetrainConstants.kSimConfig, startingPose));

    SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation.getDriveTrainSimulation());
  }

  public void startOpponentRobotSimulations() {
    try {
      instances[0] = new AIRobotInSimulation(0);
      instances[0].buildBehaviorChooser(
          PathPlannerPath.fromPathFile("AIRobotPath1"),
          Commands.none(),
          PathPlannerPath.fromPathFile("AIRobotPath2"),
          Commands.none(),
          new XboxController(1));

      opponentPosePublisher.accept(getOpponentRobotPoses());
    } catch (Exception e) {
      DriverStation.reportError(
          "Failed to load opponent robot simulation paths, error: " + e.getMessage(), false);
    }
  }

  private Command followPath(PathPlannerPath path) {
    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception ex) {
      DriverStation.reportError(
          "Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
      return Commands.none();
    }

    return new FollowPathCommand(
        path,
        driveSimulation::getActualPoseInSimulationWorld,
        driveSimulation::getActualSpeedsRobotRelative,
        (speeds, feedforwards) ->
            driveSimulation.runChassisSpeeds(speeds, new Translation2d(), false, false),
        new PPHolonomicDriveController(
            new PIDConstants(
                DrivetrainConstants.kTranslationGains.kP(),
                DrivetrainConstants.kTranslationGains.kI(),
                DrivetrainConstants.kTranslationGains.kD()),
            new PIDConstants(
                DrivetrainConstants.kHeadingGains.kP(),
                DrivetrainConstants.kHeadingGains.kI(),
                DrivetrainConstants.kHeadingGains.kD())),
        config,
        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
        this);
  }

  private Command joystickDrive(XboxController joystick) {
    // Obtain chassis speeds from joystick input
    final Supplier<ChassisSpeeds> joystickSpeeds =
        () ->
            new ChassisSpeeds(
                -joystick.getLeftY() * driveSimulation.maxLinearVelocity().in(MetersPerSecond),
                -joystick.getLeftX() * driveSimulation.maxLinearVelocity().in(MetersPerSecond),
                -joystick.getRightX() * driveSimulation.maxAngularVelocity().in(RadiansPerSecond));

    // Obtain driverstation facing for opponent driver station
    final Supplier<Rotation2d> opponentDriverStationFacing =
        () ->
            FieldMirroringUtils.getCurrentAllianceDriverStationFacing()
                .plus(Rotation2d.fromDegrees(180));

    return Commands.run(
            () -> {
              // Calculate field-centric speed from driverstation-centric speed
              final ChassisSpeeds fieldCentricSpeeds =
                  ChassisSpeeds.fromRobotRelativeSpeeds(
                      joystickSpeeds.get(),
                      FieldMirroringUtils.getCurrentAllianceDriverStationFacing()
                          .plus(Rotation2d.fromDegrees(180)));
              // Run the field-centric speed
              driveSimulation.runChassisSpeeds(fieldCentricSpeeds, new Translation2d(), true, true);
            },
            this)
        // Before the command starts, reset the robot to a position inside the field
        .beforeStarting(
            () ->
                driveSimulation.setSimulationWorldPose(
                    FieldMirroringUtils.toCurrentAlliancePose(ROBOTS_STARTING_POSITIONS[id - 1])));
  }

  /** Build the behavior chooser of this opponent robot and send it to the dashboard */
  public void buildBehaviorChooser(
      PathPlannerPath segment0,
      Command toRunAtEndOfSegment0,
      PathPlannerPath segment1,
      Command toRunAtEndOfSegment1,
      XboxController joystick) {
    SendableChooser<Command> behaviorChooser = new SendableChooser<>();
    final Supplier<Command> disable =
        () ->
            Commands.runOnce(() -> driveSimulation.setSimulationWorldPose(startingPose), this)
                .andThen(
                    Commands.runOnce(
                        () ->
                            driveSimulation.runChassisSpeeds(
                                new ChassisSpeeds(), new Translation2d(), false, false)))
                .ignoringDisable(true);

    // Option to disable the robot
    behaviorChooser.setDefaultOption("Disable", disable.get());

    // Option to auto-cycle the robot
    behaviorChooser.addOption(
        "Auto Cycle",
        getAutoCycleCommand(segment0, toRunAtEndOfSegment0, segment1, toRunAtEndOfSegment1));

    // Option to manually control the robot with a joystick
    behaviorChooser.addOption("Joystick Drive", joystickDrive(joystick));

    // Schedule the command when another behavior is selected
    behaviorChooser.onChange((Command::schedule));

    // Schedule the selected command when teleop starts
    RobotModeTriggers.teleop()
        .onTrue(Commands.runOnce(() -> behaviorChooser.getSelected().schedule()));

    // Disable the robot when the user robot is disabled
    RobotModeTriggers.disabled().onTrue(disable.get());

    SmartDashboard.putData("Opponent Robot " + id + " Behavior", behaviorChooser);
  }

  /** Get the command to auto-cycle the robot relatively */
  private Command getAutoCycleCommand(
      PathPlannerPath segment0,
      Command toRunAtEndOfSegment0,
      PathPlannerPath segment1,
      Command toRunAtEndOfSegment1) {
    final SequentialCommandGroup cycle = new SequentialCommandGroup();
    final Pose2d startingPose =
        new Pose2d(
            segment0.getStartingDifferentialPose().getTranslation(),
            segment0.getIdealStartingState().rotation());

    cycle.addCommands(followPath(segment0).andThen(toRunAtEndOfSegment0).withTimeout(10));
    cycle.addCommands(followPath(segment1).andThen(toRunAtEndOfSegment1).withTimeout(10));

    return cycle
        .repeatedly()
        .beforeStarting(
            Commands.runOnce(
                () ->
                    driveSimulation.setSimulationWorldPose(
                        FieldMirroringUtils.toCurrentAlliancePose(startingPose))));
  }

  private static Pose2d[] getRobotPoses(AIRobotInSimulation[] instances) {
    return Arrays.stream(instances)
        .map(instance -> instance.driveSimulation.getActualPoseInSimulationWorld())
        .toArray(Pose2d[]::new);
  }

  public static Pose2d[] getOpponentRobotPoses() {
    return getRobotPoses(new AIRobotInSimulation[] {instances[0]});
  }
}
