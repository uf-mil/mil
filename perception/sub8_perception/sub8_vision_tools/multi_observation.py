import numpy as np
from scipy.optimize import minimize
import image_geometry
import estimation


class MultiObservation(object):
    """
    This is NOT a bundle adjuster.

    TODO:
        Compute outliers by clustering minimum distance between rays
    """
    def __init__(self, camera_model):
        self.camera_model = camera_model
        self.image_size = (self.camera_model.cx() * 2, self.camera_model.cy() * 2)
        self.K = np.array(camera_model.fullIntrinsicMatrix(), dtype=np.float32)
        self.K_inv = np.linalg.inv(self.K)

    def column_vectorize(self, v):
        """Converts a row vector to a column vector, does nothing to a column vector"""
        return np.reshape(v, (len(v), 1))

    def lst_sqr_intersection(self, image_points, cameras):
        """Compute the least squares solution to the intersection of N rays
        Rays are parameterized as d = b - a
            Where b and a are points on the line

        COST:
            H**2 = ||c - a||**2 - (||(c - a).dot(d)||**2 / ||d||**2)
        """
        observations = map(self.get_ray, image_points)
        norm = np.linalg.norm

        def cost(c):
            t_cost = 0
            for observation, (t, R) in zip(observations, cameras):
                d = R.dot(observation)
                term_1 = norm(c - t)**2
                term_2 = (norm((c - t).dot(d))**2) / (norm(d)**2)
                t_cost += term_1 - term_2
            return t_cost

        minimization = minimize(
            method='bfgs',
            fun=cost,
            # jac=obj_jacobian,
            x0=(5.0, 5.0, 5.0),
            # x0=(self.min_thrusts + self.max_thrusts) / 2,
            # bounds=zip([-15.0, -15.0, -15.0], [15.0, 15.0, 15.0]),
            tol=1e-3
        )

        dists = []
        for observation, (t, R) in zip(observations, cameras):
            ray = R.dot(observation)
            dists.append(norm(np.cross(ray.T, minimization.x - t)))
        dists = np.array(dists)
        threshold = dists.std()
        print threshold
        if threshold < .1:  # May need to be tuned
            print "MULTI_OBS: No outliers."
            return minimization.x  # No outliers

        outliers = np.where(dists > threshold)[0][::-1]
        print "MULTI_OBS: Removing ({}) outliers.".format(len(outliers))
        for index in outliers:
            del observations[index]
            del cameras[index]

        minimization = minimize(
            method='slsqp',
            fun=cost,
            # jac=obj_jacobian,
            x0=(5.0, 5.0, 5.0),
            # x0=(self.min_thrusts + self.max_thrusts) / 2,
            bounds=zip([-15.0, -15.0, -15.0], [15.0, 15.0, 15.0]),
            tol=1e-3
        )

        return minimization.x

    def get_ray(self, observation):
        """Returns a ray in camera frame pointing towards the observation

            self.R.dot(observation) will give the ray in world frame
            self.t would be the base of the ray
        """
        obs_column = self.column_vectorize(observation)
        ray = self.K_inv.dot(np.vstack([obs_column, 1.0]))
        unit_ray = ray / np.linalg.norm(ray)
        return unit_ray


def test():
    """Here's some stupid demo code

    TODO:
        Make this an actual unit test
    """
    import sub8_ros_tools
    import time
    import rospy
    from mayavi import mlab

    rospy.init_node('test_estimation')
    q = sub8_ros_tools.Image_Subscriber('/stereo/left/image_raw')
    while(q.camera_info is None):
        time.sleep(0.1)

    camera_model = image_geometry.PinholeCameraModel()
    camera_model.fromCameraInfo(q.camera_info)
    MO = MultiObservation(camera_model)
    # K = np.array(camera_model.fullIntrinsicMatrix(), dtype=np.float32)

    real = np.array([1.0, 3.0, 7.0])
    p_wrong = .2

    projected_h = MO.K.dot(real)
    projected = projected_h[:2] / projected_h[2]

    print 'starting'
    R = np.diag([1.0, 1.0, 1.0])
    camera_t = np.array([0.0, 0.0, 0.0])
    cameras = []
    rays = []
    observations = []

    max_k = 9
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
            projected_h = MO.K.dot(np.dot(R.transpose(), real) - R.transpose().dot(camera_t))
            projected = projected_h[:2] / projected_h[2]

        obs_final = projected + np.random.normal(scale=2.0, size=2)

        cameras.append((camera_t, R))
        rays.append(MO.get_ray(obs_final))
        observations.append(obs_final)

    best_p = MO.lst_sqr_intersection(observations, cameras)
    print best_p

    mlab.points3d(*map(np.array, best_p), scale_factor=0.3)
    estimation.draw_cameras(rays, cameras)

    mlab.show()


if __name__ == '__main__':
    test()
