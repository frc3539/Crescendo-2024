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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;
import java.util.Optional;
import org.ejml.simple.SimpleMatrix;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionSubsystem extends Thread {

	// Vision Variables
	AprilTagFieldLayout aprilTagFieldLayout;

	public PhotonCamera frontLeftCam;
	Transform3d robotToFrontLeftCam = new Transform3d(new Translation3d(0.3207, 0.1397 + .06, 0.5588),
			new Rotation3d(Math.toRadians(0), Math.toRadians(-28.5), Math.toRadians(0)));

	public PhotonCamera frontRightCam;
	Transform3d robotToFrontRightCam = new Transform3d(new Translation3d(0.3207, -0.1333, 0.5588),
			new Rotation3d(Math.toRadians(0), Math.toRadians(-28.5), Math.toRadians(0)));
	public PhotonCamera backLeftCam;
	Transform3d robotToBackLeftCam = new Transform3d(new Translation3d(-0.3302, 0.2286, 0.53975),
			new Rotation3d(Math.toRadians(0), Math.toRadians(-16.5), Math.toRadians(180)));

	public PhotonCamera backRightCam;
	Transform3d robotToBackRightCam = new Transform3d(new Translation3d(-0.3302, -0.2286, 0.53975),
			new Rotation3d(Math.toRadians(0), Math.toRadians(-16.3), Math.toRadians(180)));

	public PhotonCamera noteCam;

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

		noteCam = new PhotonCamera("NoteCam");

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
		SmartDashboard.putNumberArray(key, new double[]{pose.getTranslation().getX(), pose.getTranslation().getY(),
				pose.getRotation().getRadians()});
	}

	Alliance lastAlliance = null;
	public void addVisionMeasurement(Pose2d pose, double timestampSeconds, Matrix<N3, N1> weights) {
		RobotContainer.drivetrainSubsystem.addVisionMeasurement(pose, timestampSeconds, weights);
	}

	public void log() {
		SmartDashboard.putBoolean("/Vision/BackLeft/Connected", backLeftCam.isConnected());
		SmartDashboard.putBoolean("/Vision/BackRight/Connected", backRightCam.isConnected());
		SmartDashboard.putBoolean("/Vision/FrontLeft/Connected", frontLeftCam.isConnected());
		SmartDashboard.putBoolean("/Vision/FrontRight/Connected", frontRightCam.isConnected());
		SmartDashboard.putBoolean("/Vision/NoteCam/Connected", noteCam.isConnected());

	}
	public Matrix<N3, N1> getVisionWeights(double distanceRatio, int numTargets) {
		double targetMultiplier = 1;
		double visionCutOffDistance = 4;
		distanceRatio = 0.1466 * Math.pow(1.6903, distanceRatio);
		if (numTargets == 1) {
			if (distanceRatio > visionCutOffDistance) {
				return new Matrix<N3, N1>(new SimpleMatrix(new double[]{99999, 99999, 99999}));
			}
			targetMultiplier = 3;
		}
		Matrix<N3, N1> weights = new Matrix<N3, N1>(new SimpleMatrix(new double[]{distanceRatio * targetMultiplier,
				distanceRatio * targetMultiplier, 3 + 15 * distanceRatio * targetMultiplier}));
		return weights;
	}
	public PhotonTrackedTarget getBestNote() {
		var result = noteCam.getLatestResult();
		var bestTarget = result.getBestTarget();
		return bestTarget;
	}
	@Override
	public void run() {
		/* Run as fast as possible, our signals will control the timing */
		while (true) {
			try {
				Thread.sleep(10);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			// Vision Calculations
			if (DriverStation.getAlliance().isPresent()) {
				if (DriverStation.getAlliance().get() == Alliance.Blue && lastAlliance != Alliance.Blue) {
					lastAlliance = Alliance.Blue;
					aprilTagFieldLayout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
				}
				if (DriverStation.getAlliance().get() == Alliance.Red && lastAlliance != Alliance.Red) {

					lastAlliance = Alliance.Red;
					aprilTagFieldLayout.setOrigin(OriginPosition.kRedAllianceWallRightSide);
				}
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
				if (false && resultFrontLeft.isPresent()) {
					EstimatedRobotPose camPoseFrontLeft = resultFrontLeft.get();

					double sum = 0;
					for (PhotonTrackedTarget target : camPoseFrontLeft.targetsUsed) {
						Translation2d tagPosition = aprilTagFieldLayout.getTagPose(target.getFiducialId()).get()
								.getTranslation().toTranslation2d();
						sum += resultFrontLeft.get().estimatedPose.toPose2d().getTranslation().getDistance(tagPosition);
					}
					sum /= camPoseFrontLeft.targetsUsed.size();
					double distanceRatio = sum;
					Matrix<N3, N1> weights = new Matrix<N3, N1>(new SimpleMatrix(new double[]{0.1 + 1.9 * distanceRatio,
							0.1 + 1.9 * distanceRatio, 3 + 15 * distanceRatio}));

					if (camPoseFrontLeft.timestampSeconds != frontLeftLastTimeStamp) {
						publishPose2d("/DriveTrain/FrontLeftCamPose", camPoseFrontLeft.estimatedPose.toPose2d());
						addVisionMeasurement(camPoseFrontLeft.estimatedPose.toPose2d(),
								camPoseFrontLeft.timestampSeconds, weights);

					}
					frontLeftLastTimeStamp = camPoseFrontLeft.timestampSeconds;
				}

				if (false && resultFrontRight.isPresent()) {
					EstimatedRobotPose camPoseFrontRight = resultFrontRight.get();

					double sum = 0;
					for (PhotonTrackedTarget target : camPoseFrontRight.targetsUsed) {
						Translation2d tagPosition = aprilTagFieldLayout.getTagPose(target.getFiducialId()).get()
								.getTranslation().toTranslation2d();
						sum += resultFrontRight.get().estimatedPose.toPose2d().getTranslation()
								.getDistance(tagPosition);
					}
					sum /= camPoseFrontRight.targetsUsed.size();
					double distanceRatio = sum;
					Matrix<N3, N1> weights = new Matrix<N3, N1>(new SimpleMatrix(new double[]{0.1 + 1.9 * distanceRatio,
							0.1 + 1.9 * distanceRatio, 3 + 15 * distanceRatio}));

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
					double distanceRatio = sum;
					Matrix<N3, N1> weights = getVisionWeights(distanceRatio, camPoseBackLeft.targetsUsed.size());

					if (camPoseBackLeft.timestampSeconds != backLeftLastTimeStamp) {
						publishPose2d("/DriveTrain/BackLeftCamPose", camPoseBackLeft.estimatedPose.toPose2d());
						SmartDashboard.putString("/Vision/BackLeftWeights", weights.toString());
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
					double distanceRatio = sum;
					Matrix<N3, N1> weights = getVisionWeights(distanceRatio, camPoseBackRight.targetsUsed.size());

					if (camPoseBackRight.timestampSeconds != backRightLastTimeStamp) {
						publishPose2d("/DriveTrain/BackRightCamPose", camPoseBackRight.estimatedPose.toPose2d());
						SmartDashboard.putString("/Vision/BackRightWeights", weights.toString());
						RobotContainer.drivetrainSubsystem.addVisionMeasurement(
								camPoseBackRight.estimatedPose.toPose2d(), camPoseBackRight.timestampSeconds, weights);
					}
					backRightLastTimeStamp = camPoseBackRight.timestampSeconds;
				}
			}
		}
	}
}
