/*
 * MIT License
 *
 * Copyright (c) PhotonVision
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package lib.frc706.cyberlib.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import java.io.UncheckedIOException;
import java.util.Optional;
import java.util.List;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PhotonCameraWrapper {
	PhotonCamera photonCamera;
	PhotonPoseEstimator photonPoseEstimator; //TODO: make this private again
	// and move all the stuff from swervesubsystem into here where it's supposed to be

	public PhotonCameraWrapper(String cameraName, Transform3d robotToCam) {
		// Change the name of your camera here to whatever it is in the PhotonVision UI.
		photonCamera = new PhotonCamera(cameraName);
		try {
			// Attempt to load the AprilTagFieldLayout that will tell us where the tags are
			// on the field.
			AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
			// Create pose estimator
			photonPoseEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCam);
			photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
			
		} catch (UncheckedIOException e) {
			// The AprilTagFieldLayout failed to load. We won't be able to estimate poses if
			// we don't know where the tags are.
			DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
			photonPoseEstimator = null;
		}
	}

	public AprilTagFieldLayout getFieldLayout() {
		return photonPoseEstimator.getFieldTags();
	}

	/**
	 * @param prevEstimatedRobotPose The current best guess at robot pose
	 * @return an EstimatedRobotPose with an estimated pose, the timestamp, and
	 *         targets used to create the estimate
	 */
	public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
		if (photonPoseEstimator == null) {
			// The field layout failed to load, so we cannot estimate poses.
			return Optional.empty();
		}
		photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
		return photonPoseEstimator.update(photonCamera.getLatestResult());
	}

	public List<PhotonTrackedTarget> getTags() {
		return photonCamera.getLatestResult().getTargets();
	}
}