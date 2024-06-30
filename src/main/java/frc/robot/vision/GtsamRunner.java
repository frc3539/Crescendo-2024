package frc.robot.vision;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Twist3d;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.RobotContainer;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.vision.photonvision.gtsam.GtsamInterface;
import frc.robot.vision.photonvision.gtsam.TagDetection;

public class GtsamRunner {
	public GtsamInterface iface;
	private PhotonCamera backLeftCam;
	private PhotonCamera backRightCam;

	public GtsamRunner() {
		this.backLeftCam = new PhotonCamera("BackLeft");
		this.backRightCam = new PhotonCamera("BackRight");
		this.iface = new GtsamInterface(List.of("BackLeft", "BackRight"));
	}

	PhotonPipelineResult lastLeftResult = new PhotonPipelineResult();
	PhotonPipelineResult lastRightResult = new PhotonPipelineResult();

	public void update() {
		var now = RobotController.getFPGATime();

		// TODO figure out how to extract a odom-only twist from the drivetrain
		iface.sendOdomUpdate(now, new Twist3d());

		var newLeft = backLeftCam.getLatestResult();
		var newRight = backRightCam.getLatestResult();

		if (newLeft.getTimestampSeconds() != lastLeftResult.getTimestampSeconds()) {
			lastLeftResult = newLeft;

			var tags = new ArrayList<TagDetection>();
			for (var result : newLeft.getTargets()) {
				tags.add(new TagDetection(result.getFiducialId(), result.getDetectedCorners()));
			}

			iface.sendVisionUpdate("BackLeft", (long) (newLeft.getTimestampSeconds() * 1e6), tags,
					VisionSubsystem.robotToBackLeftCam);
		}

		if (newRight.getTimestampSeconds() != lastRightResult.getTimestampSeconds()) {
			lastRightResult = newRight;

			var tags = new ArrayList<TagDetection>();
			for (var result : newRight.getTargets()) {
				tags.add(new TagDetection(result.getFiducialId(), result.getDetectedCorners()));
			}

			iface.sendVisionUpdate("BackRight", (long) (newRight.getTimestampSeconds() * 1e6), tags,
					VisionSubsystem.robotToBackRightCam);
		}
	}

	public void sendTagLayout() {
		iface.sendLayout(AprilTagFields.k2024Crescendo.loadAprilTagLayoutField());
	}

	public void sendInitialGuess() {
		iface.sendGuess(RobotController.getFPGATime(), new Pose3d(RobotContainer.drivetrainSubsystem.getPose2d()));
		iface.setCamIntrinsics("BackLeft", backLeftCam.getCameraMatrix(), backLeftCam.getDistCoeffs());
		iface.setCamIntrinsics("BackRight", backRightCam.getCameraMatrix(), backRightCam.getDistCoeffs());
	}
}
