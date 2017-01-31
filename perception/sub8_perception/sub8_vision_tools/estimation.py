from __future__ import division
import image_geometry
import numpy as np
from scipy import stats
from mayavi import mlab


class ProjectionParticleFilter(object):
    _debug = True

    def __init__(self, camera_model, num_particles=10000, max_Z=25, observation_noise=5.0, jitter_prob=0.2, salting=0.1,
                 aggressive=True):
        """A particle filter for estimating pose from multiple camera views

        ppf = ProjectionParticleFitler(image_geometry.PinholeCameraModel(msg.cameraInfo))
        for k in range(5):
            centroid = my_object_finder.find(images[k])
            ppf.observe(centroid)

        Calling ppf.observe returns the current best estimate and the particle covariance
        Note: Covariance is not a complete measure of certainty

        :param camera_model: ros image_geometry PinholeCameraModel, must be initialized
        :param num_particles: How many particles to use, we can operate comfortably with 100,000
        :param max_Z: The maximum depth to search in
        :param observation_noise: The standard-deviation of the noise (in units of pixels)
        :param jitter_prob: Probability of a completely random measurement
        :param salting: std-def of how much we'll spice up the estimates!
        :param aggressive: Use a gaussian error distributions instead of error**2, much more aggressive
            may sometimes throw probability did not sum to one errors (because it is too aggressive!)

        Implementation Notes:
            - The measurement pdf I am using is not z ~ N(K * x | sig**2, mu)
                - It is instead the p(z | x) + p(z | total randomness)
                - This makes us much much more resilient to random error

        TODO:
            - If probabilities fall below something reasonable, more frequently do reset_to_ray
                - In some cases, reset the worst 25% of particles to ray
        """
        assert isinstance(camera_model, image_geometry.PinholeCameraModel), "Must be pinhole camera"
        self.aggressive = aggressive
        self.min_Z = 0.0
        self.max_Z = max_Z
        self.salt = salting
        self.observation_noise = observation_noise
        self.jitter = jitter_prob
        self.camera_model = camera_model

        self.cameras = []
        self.observations = []

        self.image_size = (self.camera_model.cx() * 2, self.camera_model.cy() * 2)
        self.K = np.array(camera_model.fullIntrinsicMatrix(), dtype=np.float32)
        self.K_inv = np.linalg.inv(self.K)

        self.num_particles = num_particles
        self.jitter_pdf = stats.uniform(loc=0.0, scale=max(self.image_size))

        # This line doesn't *necessarily* do something
        self.particles = np.random.uniform((-25, -25, self.min_Z), (25, 25, self.max_Z), size=(self.num_particles, 3)).transpose()
        self.weights = np.ones(self.num_particles)
        self.weights = self.weights / np.sum(self.weights)
        self.t = np.zeros(3)
        self.R = np.diag([1.0, 1.0, 1.0])
        self.on_first_observation = True

    def get_best(self):
        """Return the best point and the diagonal of the covariance matrix of our particles

            Only the diagonal has interesting stuff, you should mostly be concerned with the Z covariance
                being smaller than the radius of your target
        """
        ind = np.argmax(self.weights)
        weighted_avg = np.average(self.particles, axis=1, weights=self.weights)
        # return self.particles[:, ind], np.diag(np.cov(self.particles))
        # This allows us to handle a fair bit of random noise
        return weighted_avg, np.diag(np.cov(self.particles))

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
            np.all(projected < self.column_vectorize(self.image_size), axis=0)
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
        if self._debug:
            self.cameras.append((t, R))

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
        if self._debug:
            self.observations.append(unit_ray)

        if weights is None:
            # This favors distant particles, which is not what we want
            # range_distribution = np.random.uniform(self.min_Z, self.max_Z, size=self.num_particles)

            range_distribution = np.random.normal(loc=self.max_Z / 2, scale=self.max_Z / 6, size=self.num_particles)

            self.particles = self.t + (range_distribution * unit_ray)
            self.particles += np.random.normal(scale=(self.salt, self.salt, self.salt), size=(self.num_particles, 3)).transpose()

        else:
            # Doesn't yet work
            average_p = 0.2 * np.average(weights)
            self.particles[:, weights < average_p] = np.reshape(-self.t, (3, 1)) + np.random.uniform(
                self.min_Z,
                self.max_Z,
                size=self.particles[:, weights < average_p]
            ) * unit_ray

            self.particles[:, weights < average_p] += np.random.normal(
                scale=(self.salt, self.salt, self.salt), size=(self.num_particles, 3)
            ).transpose()

    def observe(self, observation):
        """Measurement-update, take a single-pixel observation

        :param observation: observation should be a column vector, should be pixel coordinate

        - Consume an observation and use it to estimate the target pose!
        - Does nothing if the observation is shitty (or if it doesn't help)

        TODO
            - Must shuffle instead of relying on state transition distribution
        """
        observation = self.column_vectorize(observation)
        if self.on_first_observation:
            self._reset_to_ray(observation)
            self.on_first_observation = False

        # Compute expected z
        particles = self._transform_pts(self.t, self.R)
        expected_obs_h = self.K.dot(particles)
        expected_obs = expected_obs_h[:2, :] / expected_obs_h[2, :]

        # Check how many particles are in the field of view, if we have a problem, don't try
        infront = np.sum(self.in_fov(particles))
        if infront < 5:
            # TODO: Do a partial ray-reset
            return

        # Compute p(z | x)
        error = np.linalg.norm(expected_obs - observation, axis=0)
        if self.aggressive:
            # TODO: logpdf instead
            z_prob = stats.uniform.pdf(particles[2, :], loc=0.0, scale=self.max_Z)
            reprojection_prob = stats.norm.pdf(error, scale=self.observation_noise)

            # p(z | x) = (1 - p_outlier ) * p(err | sigma) + (p_outlier * p(err | random))
            prob = ((1 - self.jitter) * (z_prob * reprojection_prob)) + (self.jitter * self.jitter_pdf.pdf(error))

        else:
            prob = ((1 - self.jitter) * np.power(error ** 2, -1)) + (self.jitter * self.jitter_pdf.pdf(error))

        # Normalize because we suck
        normalized_prob = prob / np.sum(prob)

        # Importance sampling
        to_sample = int(self.num_particles)
        choice_indices = np.random.choice(
            a=np.arange(particles.shape[1]),
            p=normalized_prob,
            size=to_sample,
            replace=True,
        )
        choices = self.particles[:, choice_indices]

        # Update our particles
        self.particles = choices + np.random.normal(
            scale=(self.salt, self.salt, self.salt),
            size=(self.num_particles, 3)
        ).transpose()

        # Update our weights
        new_weights = normalized_prob[choice_indices]
        weights = new_weights
        self.weights = weights / np.sum(weights)
        return self.get_best()

    def visualize(self):
        draw_particles(self)
        draw_cameras(self.observations, self.cameras)
        mlab.show()


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
    """Here's some stupid demo code
    """
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
    p_wrong = 0.4

    projected_h = ppf.K.dot(real)
    projected = projected_h[:2] / projected_h[2]

    print 'starting'
    R = np.diag([1.0, 1.0, 1.0])
    camera_t = np.array([0.0, 0.0, 0.0])
    cameras = []
    observations = []

    cov = np.array([100, 100, 100])
    max_k = 15
    for k in range(max_k):
        if k < 1:
            camera_t = np.array([0.0, -8.0, 7.0])
            R = np.array([
                [1.0, 0.0, 0.0],
                [0.0, 0.0, 1.0],
                [0.0, -1.0, 0.0],
            ])

        else:
            R = np.diag([1.0, 1.0, 1.0])
            camera_t = np.hstack([(np.random.random(2) - 0.5) * 5, 0.0])

        if (np.random.random() < p_wrong) and (k > 1):
            print "Doing a random observation"
            projected = np.random.random(2) * np.array([640., 480.])
        else:
            projected_h = ppf.K.dot(np.dot(R.transpose(), real) - R.transpose().dot(camera_t))
            projected = projected_h[:2] / projected_h[2]

        obs_final = projected + np.random.normal(scale=5.0, size=2)

        cameras.append((camera_t, R))
        observations.append(ppf.get_ray(obs_final))
        # draw_cameras(observations, cameras)
        # draw_particles(ppf, color_hsv=((k + 1) / (max_k + 1), 0.7, 0.8), scale=0.1 * ((k + 1) / max_k))
        # mlab.show()

        # Interpolate along hsv to get a cool heatmap effect

        ppf.set_pose(camera_t, R)
        ppf.observe(np.reshape(obs_final, (2, 1)))

        best_v, cov = ppf.get_best()

        print 'best', best_v
        print 'cov', cov

    draw_cameras(observations, cameras)
    draw_particles(ppf, color_hsv=((k + 1) / (max_k + 1), 0.7, 0.8), scale=0.1 * ((k + 1) / max_k))

    mlab.show()


if __name__ == '__main__':
    main()
