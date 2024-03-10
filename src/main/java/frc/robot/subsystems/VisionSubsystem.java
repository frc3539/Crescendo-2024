package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.RobotContainer;
import java.util.Optional;
import org.ejml.simple.SimpleMatrix;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionSubsystem extends Thread {

	// Vision Variables
	AprilTagFieldLayout aprilTagFieldLayout;

	public PhotonCamera frontLeftCam;
	Transform3d robotToFrontLeftCam = new Transform3d(
			new Translation3d(-0.1746 - .07 + .02 + 0.08, 0.2885 + 0.05 - .03, 0.5461),
			new Rotation3d(Math.toRadians(0), 0, Math.toRadians(0)));

	public PhotonCamera frontRightCam;
	Transform3d robotToFrontRightCam = new Transform3d(
			new Translation3d(-0.1746 + .05 + .02, -0.2885 - .1 + .05, 0.5544),
			new Rotation3d(Math.toRadians(0), 0, Math.toRadians(0)));
	public PhotonCamera backLeftCam;
	Transform3d robotToBackLeftCam = new Transform3d(new Translation3d(-0.3302, 0.2286, 0.53975),
			new Rotation3d(Math.toRadians(0), Math.toRadians(-16.5), Math.toRadians(180)));

	public PhotonCamera backRightCam;
	Transform3d robotToBackRightCam = new Transform3d(new Translation3d(-0.3302, -0.2286, 0.53975),
			new Rotation3d(Math.toRadians(0), Math.toRadians(-16.3), Math.toRadians(180)));

	PhotonPoseEstimator frontLeftPhotonPoseEstimator;
	PhotonPoseEstimator frontRightPhotonPoseEstimator;
	PhotonPoseEstimator backLeftPhotonPoseEstimator;
	PhotonPoseEstimator backRightPhotonPoseEstimator;
	Optional<EstimatedRobotPose> resultFrontLeft;
	Optional<EstimatedRobotPose> resultFrontRight;
	Optional<EstimatedRobotPose> resultBackLeft;
	Optional<EstimatedRobotPose> resultBackRight;
	boolean useVision = true;
	double frontLeftLastTimeStamp = 0;
	double frontRightLastTimeStamp = 0;
	double backLeftLastTimeStamp = 0;
	double backRightLastTimeStamp = 0;

	double visionRatio = 10;

	public VisionSubsystem() {
		super();
		try {
			aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);

		} catch (Exception e) {
			System.out.println("ERROR Loading April Tag DATA");
			aprilTagFieldLayout = null;
		}

		frontLeftCam = new PhotonCamera("FrontLeft");
		frontLeftPhotonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
				PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, frontLeftCam, robotToFrontLeftCam);

		frontRightCam = new PhotonCamera("FrontRight");
		frontRightPhotonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
				PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, frontRightCam, robotToFrontRightCam);

		backLeftCam = new PhotonCamera("BackLeft");
		backLeftPhotonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
				PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, backLeftCam, robotToBackLeftCam);

		backRightCam = new PhotonCamera("BackRight");
		backRightPhotonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
				PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, backRightCam, robotToBackRightCam);

		setVisionWeights(.2, .2, 10);
	}

	// Vision Methods
	public Optional<EstimatedRobotPose> getEstimatedFrontLeftGlobalPose() {
		return frontLeftPhotonPoseEstimator.update();
	}

	public Optional<EstimatedRobotPose> getEstimatedFrontRightGlobalPose() {
		return frontRightPhotonPoseEstimator.update();
	}

	public Optional<EstimatedRobotPose> getEstimatedBackLeftGlobalPose() {
		return backLeftPhotonPoseEstimator.update();
	}

	public Optional<EstimatedRobotPose> getEstimatedBackRightGlobalPose() {
		return backRightPhotonPoseEstimator.update();
	}

	public void useVision(boolean useVision) {
		this.useVision = useVision;
	}

	public void setVisionWeights(double visionX, double visionY, double visionDeg) {
		RobotContainer.drivetrainSubsystem
				.setVisionMeasurementStdDevs(VecBuilder.fill(visionX, visionY, Units.degreesToRadians(visionDeg)));
	}

	public static void publishPose2d(String key, Pose2d pose) {
		Logger.recordOutput(key, new double[]{pose.getTranslation().getX(), pose.getTranslation().getY(),
				pose.getRotation().getRadians()});
	}

	Alliance lastAlliance = null;
	public void addVisionMeasurement(Pose2d pose, double timestampSeconds, Matrix<N3, N1> weights) {
		RobotContainer.drivetrainSubsystem.addVisionMeasurement(pose, timestampSeconds, weights);
	}

	public void log() {
		Logger.recordOutput("/Vision/BackLeft/Connected", backLeftCam.isConnected());
		Logger.recordOutput("/Vision/BackRight/Connected", backRightCam.isConnected());
		Logger.recordOutput("/Vision/FrontLeft/Connected", frontLeftCam.isConnected());
		Logger.recordOutput("/Vision/FrontRight/Connected", frontRightCam.isConnected());

	}
	@Override
	public void run() {
		/* Run as fast as possible, our signals will control the timing */
		while (true) {
			// Vision Calculations
			if (DriverStation.getAlliance().isPresent()) {
				if (DriverStation.getAlliance().get() == Alliance.Blue && lastAlliance != Alliance.Blue)
					aprilTagFieldLayout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
				if (DriverStation.getAlliance().get() == Alliance.Red && lastAlliance != Alliance.Red)
					aprilTagFieldLayout.setOrigin(OriginPosition.kRedAllianceWallRightSide);
			}

			frontLeftPhotonPoseEstimator.setFieldTags(aprilTagFieldLayout);
			frontRightPhotonPoseEstimator.setFieldTags(aprilTagFieldLayout);
			backLeftPhotonPoseEstimator.setFieldTags(aprilTagFieldLayout);
			backRightPhotonPoseEstimator.setFieldTags(aprilTagFieldLayout);

			this.resultFrontLeft = getEstimatedFrontLeftGlobalPose();
			this.resultFrontRight = getEstimatedFrontRightGlobalPose();
			this.resultBackLeft = getEstimatedBackLeftGlobalPose();
			this.resultBackRight = getEstimatedBackRightGlobalPose();

			if (useVision) {
				if (resultFrontLeft.isPresent()) {
					EstimatedRobotPose camPoseFrontLeft = resultFrontLeft.get();

					double sum = 0;
					for (PhotonTrackedTarget target : camPoseFrontLeft.targetsUsed) {
						Translation2d tagPosition = aprilTagFieldLayout.getTagPose(target.getFiducialId()).get()
								.getTranslation().toTranslation2d();
						sum += resultFrontLeft.get().estimatedPose.toPose2d().getTranslation().getDistance(tagPosition);
					}
					sum /= camPoseFrontLeft.targetsUsed.size();
					double distanceRatio = sum / visionRatio;
					Matrix<N3, N1> weights = new Matrix<N3, N1>(new SimpleMatrix(new double[]{0.1 + 1.9 * distanceRatio,
							0.1 + 1.9 * distanceRatio, 5 + 25 * distanceRatio}));

					if (camPoseFrontLeft.timestampSeconds != frontLeftLastTimeStamp) {
						publishPose2d("/DriveTrain/FrontLeftCamPose", camPoseFrontLeft.estimatedPose.toPose2d());
						addVisionMeasurement(camPoseFrontLeft.estimatedPose.toPose2d(),
								camPoseFrontLeft.timestampSeconds, weights);

					}
					frontLeftLastTimeStamp = camPoseFrontLeft.timestampSeconds;
				}

				if (resultFrontRight.isPresent()) {
					EstimatedRobotPose camPoseFrontRight = resultFrontRight.get();

					double sum = 0;
					for (PhotonTrackedTarget target : camPoseFrontRight.targetsUsed) {
						Translation2d tagPosition = aprilTagFieldLayout.getTagPose(target.getFiducialId()).get()
								.getTranslation().toTranslation2d();
						sum += resultFrontRight.get().estimatedPose.toPose2d().getTranslation()
								.getDistance(tagPosition);
					}
					sum /= camPoseFrontRight.targetsUsed.size();
					double distanceRatio = sum / visionRatio;
					Matrix<N3, N1> weights = new Matrix<N3, N1>(new SimpleMatrix(new double[]{0.1 + 1.9 * distanceRatio,
							0.1 + 1.9 * distanceRatio, 5 + 25 * distanceRatio}));

					if (camPoseFrontRight.timestampSeconds != frontRightLastTimeStamp) {
						publishPose2d("/DriveTrain/FrontRightCamPose", camPoseFrontRight.estimatedPose.toPose2d());
						RobotContainer.drivetrainSubsystem.addVisionMeasurement(
								camPoseFrontRight.estimatedPose.toPose2d(), camPoseFrontRight.timestampSeconds,
								weights);

					}
					frontRightLastTimeStamp = camPoseFrontRight.timestampSeconds;
				}

				if (resultBackLeft.isPresent()) {
					EstimatedRobotPose camPoseBackLeft = resultBackLeft.get();

					double sum = 0;
					for (PhotonTrackedTarget target : camPoseBackLeft.targetsUsed) {
						Translation2d tagPosition = aprilTagFieldLayout.getTagPose(target.getFiducialId()).get()
								.getTranslation().toTranslation2d();
						sum += resultBackLeft.get().estimatedPose.toPose2d().getTranslation().getDistance(tagPosition);
					}
					sum /= camPoseBackLeft.targetsUsed.size();
					double distanceRatio = sum / visionRatio;
					Matrix<N3, N1> weights = new Matrix<N3, N1>(new SimpleMatrix(new double[]{0.1 + 1.9 * distanceRatio,
							0.1 + 1.9 * distanceRatio, 5 + 25 * distanceRatio}));

					if (camPoseBackLeft.timestampSeconds != backLeftLastTimeStamp) {
						publishPose2d("/DriveTrain/BackLeftCamPose", camPoseBackLeft.estimatedPose.toPose2d());
						Logger.recordOutput("/Vision/BackLeftWeights", weights.toString());
						RobotContainer.drivetrainSubsystem.addVisionMeasurement(
								camPoseBackLeft.estimatedPose.toPose2d(), camPoseBackLeft.timestampSeconds, weights);

					}
					backLeftLastTimeStamp = camPoseBackLeft.timestampSeconds;
				}

				if (resultBackRight.isPresent()) {
					EstimatedRobotPose camPoseBackRight = resultBackRight.get();

					double sum = 0;
					for (PhotonTrackedTarget target : camPoseBackRight.targetsUsed) {
						Translation2d tagPosition = aprilTagFieldLayout.getTagPose(target.getFiducialId()).get()
								.getTranslation().toTranslation2d();
						sum += resultBackRight.get().estimatedPose.toPose2d().getTranslation().getDistance(tagPosition);
					}
					sum /= camPoseBackRight.targetsUsed.size();
					double distanceRatio = sum / visionRatio;
					Matrix<N3, N1> weights = new Matrix<N3, N1>(new SimpleMatrix(new double[]{0.1 + 1.9 * distanceRatio,
							0.1 + 1.9 * distanceRatio, 5 + 25 * distanceRatio}));

					if (camPoseBackRight.timestampSeconds != backRightLastTimeStamp) {
						publishPose2d("/DriveTrain/BackRightCamPose", camPoseBackRight.estimatedPose.toPose2d());
						Logger.recordOutput("/Vision/BackRightWeights", weights.toString());
						RobotContainer.drivetrainSubsystem.addVisionMeasurement(
								camPoseBackRight.estimatedPose.toPose2d(), camPoseBackRight.timestampSeconds, weights);

					}
					backRightLastTimeStamp = camPoseBackRight.timestampSeconds;
				}
			}
		}
	}
}
