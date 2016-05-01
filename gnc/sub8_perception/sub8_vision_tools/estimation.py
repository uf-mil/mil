from __future__ import division
import image_geometry
import numpy as np
from scipy import stats
from mayavi import mlab


class ProjectionParticleFilter(object):
    def __init__(self, camera_model, num_particles=10000, max_Z=25, aggressive=True):
        """A particle filter for estimating pose from multiple camera views

        ppf = ProjectionParticleFitler(image_geometry.PinholeCameraModel(msg.cameraInfo))
        for k in range(5):
            centroid = my_object_finder.find(images[k])
            ppf.observe(centroid)

        Calling ppf.observe returns
        Note: Covariance is not a complete measure of certainty

        :param camera_model: ros image_geometry PinholeCameraModel, must be initialized
        :param num_particles: How many particles to use, we can operate comfortably with 100,000
        :param max_Z: The maximum depth to search in
        :param aggressive: Use a gaussian error distributions instead of error**2, much more aggressive
            may sometimes throw probability did not sum to one errors (because it is too aggressive!)

        TODO:
            - If probabilities fall below something reasonable, more frequently do reset_to_ray
                - In some cases, reset the worst 25% of particles to ray
        """
        assert isinstance(camera_model, image_geometry.PinholeCameraModel), "Must be pinhole camera"
        self.aggressive = aggressive
        self.min_Z = 0.0
        self.max_Z = max_Z

        self.camera_model = camera_model
        self.K = np.array(camera_model.fullIntrinsicMatrix(), dtype=np.float32)
        self.K_inv = np.linalg.inv(self.K)

        self.num_particles = num_particles
        self.particles = np.random.uniform((-25, -25, self.min_Z), (25, 25, self.max_Z), size=(self.num_particles, 3)).transpose()
        self.weights = np.ones(self.num_particles)
        self.weights = self.weights / np.sum(self.weights)
        self.t = np.zeros(3)
        self.R = np.diag([1.0, 1.0, 1.0])
        self.first_observation = True

    def get_best(self):
        """Return the best point and the diagonal of the covariance matrix of our particles

            Only the diagonal has interesting stuff, you should mostly be concerned with the Z covariance
                being smaller than the radius of your target
        """
        ind = np.argmax(self.weights)
        return self.particles[:, ind], np.diag(np.cov(self.particles))

    def column_vectorize(self, v):
        """Converts a row vector to a column vector, does nothing to a column vector"""
        return np.reshape(v, (len(v), 1))

    def in_fov(self, pts):
        """Return a mask of points that are in the field-of-view of the camera"""
        projected_h = self.K.dot(pts)
        not_infinite = projected_h[2, :] > 1e-5
        projected = projected_h[:2, :] / projected_h[2, :]

        # I acknowledge that the image edges are NOT guaranteed to be at (2cx, 2cy)
        # but PinholeCameraModel doesn't contain resolution information :/
        in_image = np.logical_and(
            np.all(projected > self.column_vectorize((0.0, 0.0)), axis=0),
            np.all(projected < self.column_vectorize((self.camera_model.cx() * 2, self.camera_model.cy() * 2)), axis=0)
        )
        in_front = pts[2, :] > 0
        in_fov = np.logical_and(np.logical_and(
            in_image, in_front
        ), not_infinite)

        return in_fov

    def set_pose(self, t, R):
        """Set the pose of the camera for which our observation is valid
        """
        assert R.shape == (3, 3), "Rotation must be 3x3 rotation matrix"
        assert len(t) == 3, "Translation vector must be of length 3"
        self.t = self.column_vectorize(t)
        self.R = R

    def _transform_pts(self, t, R):
        """Supply a transform to the camera frame"""
        particles = np.copy(np.dot(R.transpose(), self.particles) - R.transpose().dot(t))
        return particles

    def get_ray(self, observation):
        """Returns a ray in camera frame pointing towards the observation

            self.R.dot(observation) will give the ray in world frame
            self.t would be the base of the ray
        """
        obs_column = self.column_vectorize(observation)
        ray = self.K_inv.dot(np.vstack([obs_column, 1.0]))
        unit_ray = ray / np.linalg.norm(ray)
        return unit_ray

    def _reset_to_ray(self, observation, weights=None):
        """Rest all of the particles to lie along the observation ray
        :param observation: A column vector representing the pixel coordinates
        :param weights: dims=[num_particles], particle weight, higher is better
        We *know* that the true point lies along a ray from the camera center through the focal plane at the observation point
         So, if we reset all of our particles to lie along that ray, we save convergence time substantially
        """
        unit_ray = self.R.dot(self.get_ray(observation))

        if weights is None:
            self.particles = self.t + (np.random.uniform(self.min_Z, self.max_Z, size=self.num_particles) * unit_ray)
            self.particles += np.random.normal(scale=(0.1, 0.1, 0.1), size=(self.num_particles, 3)).transpose()

        else:
            average_p = 0.2 * np.average(weights)
            self.particles[:, weights < average_p] = np.reshape(-self.t, (3, 1)) + np.random.uniform(
                self.min_Z,
                self.max_Z,
                size=self.particles[:, weights < average_p]
            ) * unit_ray

        self.first_observation = False

    def observe(self, observation):
        """Measurement-update, take a single-pixel observation

        :param observation: observation should be a column vector, should be pixel coordinate

        - Consume an observation and use it to estimate the target pose!
        - Does nothing if the observation is shitty (or if it doesn't help)

        TODO
            - Must shuffle instead of relying on state transition distribution
        """
        if self.first_observation:
            self._reset_to_ray(observation)
            self.first_observation = False

        # compute p(z | x)
        particles = self._transform_pts(self.t, self.R)
        expected_obs_h = self.K.dot(particles)

        # Check how many particles are in the field of view, if we have a problem, don't try
        infront = np.sum(self.in_fov(particles))
        if infront < 5:
            # TODO: Do a partial ray-reset
            return

        expected_obs = expected_obs_h[:2, :] / expected_obs_h[2, :]

        error = np.linalg.norm(expected_obs - observation, axis=0)

        if self.aggressive:
            prob = stats.norm.pdf(error, scale=5.0) * stats.uniform.pdf(particles[2, :], loc=0.0, scale=self.max_Z)
        else:
            prob = np.power(error ** 2, -1)

        normalized_prob = prob / np.sum(prob)

        to_sample = int(self.num_particles)
        choice_indices = np.random.choice(
            a=np.arange(particles.shape[1]),
            p=normalized_prob,
            size=to_sample,
            replace=True,
        )

        choices = self.particles[:, choice_indices]

        self.particles = choices + np.random.normal(scale=(0.01, 0.01, 0.01), size=(self.num_particles, 3)).transpose()

        new_weights = normalized_prob[choice_indices]
        weights = new_weights
        self.weights = weights / np.sum(weights)


def draw_particles(ppf, color_hsv=(1.0, 0.2, 1.0), scale=0.05):
    """Draw the particles we are tracking! Whoa!
    """
    mlab.points3d(
        ppf.particles[0, :][::20],
        ppf.particles[1, :][::20],
        ppf.particles[2, :][::20],
        scale_factor=scale,
        color=color_hsv,
        colormap='hsv'
    )
    mlab.axes()


def draw_cameras(observations, cameras):
    """Draw the cameras!"""
    for observation, camera in zip(observations, cameras):
        t, R = camera
        draw_camera(t, R)
        draw_line(t, t + (R.dot(observation.flatten()) * 15), color=(0.0, 1.0, 0.0))


def draw_camera(t, R):
    """Draw a single camera"""
    colors = (
        (1.0, 0.0, 0.0),
        (0.0, 1.0, 0.0),
        (0.0, 0.0, 1.0),
    )
    for line, color in zip(R, colors):
        draw_line(t, t + line, color=color)

    mlab.points3d(
        [t[0]],
        [t[1]],
        [t[2]],
        color=(0.0, 1.0, 0.0),
        scale_factor=0.3
    )


def draw_line(pt_1, pt_2, color=(1.0, 0.0, 0.0)):
    """Draw a line between two points"""
    mlab.plot3d(
        np.hstack([pt_1[0], pt_2[0]]),
        np.hstack([pt_1[1], pt_2[1]]),
        np.hstack([pt_1[2], pt_2[2]]),
        color=color
    )


def main():
    import sub8_ros_tools
    import time
    import rospy
    rospy.init_node('test_estimation')
    q = sub8_ros_tools.Image_Subscriber('/stereo/left/image_rect_color')
    while(q.camera_info is None):
        time.sleep(0.1)

    camera_model = image_geometry.PinholeCameraModel()
    camera_model.fromCameraInfo(q.camera_info)

    ppf = ProjectionParticleFilter(camera_model, 100000)
    real = np.array([1.0, 3.0, 7.0])
    projected_h = ppf.K.dot(real)
    projected = projected_h[:2] / projected_h[2]

    print 'starting'
    R = np.diag([1.0, 1.0, 1.0])
    camera_t = np.array([0.0, 0.0, 0.0])
    cameras = []
    observations = []

    max_k = 3
    for k in range(max_k):

        if k < 1:
            camera_t = np.array([0.0, -8.0, 7.0])
            R = np.array([
                [1.0, 0.0, 0.0],
                [0.0, 0.0, 1.0],
                [0.0, -1.0, 0.0],
            ])

        else:
            camera_t = np.hstack([(np.random.random(2) - 0.5) * 5, 0.0])

        projected_h = ppf.K.dot(np.dot(R.transpose(), real) - R.transpose().dot(camera_t))
        projected = projected_h[:2] / projected_h[2]

        obs_final = projected + np.random.normal(scale=3.0, size=2)

        cameras.append((camera_t, R))
        observations.append(ppf.get_ray(obs_final))
        draw_cameras(observations, cameras)
        mlab.show()
        draw_particles(ppf, color_hsv=((k + 1) / (max_k + 1), 0.7, 0.8), scale=0.1 * ((k + 1) / max_k))

        ppf.set_pose(camera_t, R)
        ppf.observe(np.reshape(obs_final, (2, 1)))

    print 'cov:', np.diag(np.cov(ppf.particles))
    best = ppf.get_best()

    print 'best'
    print best, best / np.linalg.norm(best)
    print real, real / np.linalg.norm(real)
    draw_cameras(observations, cameras)
    draw_particles(ppf, color_hsv=((k + 1) / (max_k + 1), 0.7, 0.8), scale=0.1 * ((k + 1) / max_k))

    mlab.show()


if __name__ == '__main__':
    main()
