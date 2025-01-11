package frc.robot.subsystems.arm;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.epilogue.logging.LogBackedSendableBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;


@Logged
public class ArmVisualizer {
  private final Mechanism2d mechanism;

  @NotLogged
  private final MechanismLigament2d arm;

  @Logged(name = "ArmPose")
  private Pose3d armPose = new Pose3d(); 

  public ArmVisualizer(Color color) {
    mechanism = new Mechanism2d(3.0, 3.0, new Color8Bit(Color.kWhite));
    MechanismRoot2d root = mechanism.getRoot("pivot", 1.0, 0.4);
    arm = new MechanismLigament2d("arm", ArmConstants.LENGTH_METERS, 20.0, 6, new Color8Bit(color));
    root.append(arm);

    // mechanism.initSendable(new LogBackedSendableBuilder(Epilogue.getConfig().dataLogger));
  }

  // Update arm visualizer with current arm angle
  public void update(Rotation2d angle) {
    // Log Mechanism2d
    arm.setAngle(angle);

    // Log 3D poses
    armPose = new Pose3d(new Translation3d(-0.20447, 0, 0.29), new Rotation3d(90 * Math.PI / 180, angle.getRadians(), 180 * Math.PI / 180));
  }
}
