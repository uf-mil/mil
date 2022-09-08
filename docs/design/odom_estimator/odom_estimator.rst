``odom_estimator``
==================

``odom_estimator`` is an `unscented Kalman filter <https://en.wikipedia.org/wiki/Kalman_filter#Unscented_Kalman_filter>`_ that fuses accelerometer, gyroscope, magnetometer, DVL, and depth sensor measurements into an estimate of a moving platform's pose.

If you want to understand the math behind ``odom_estimator``, I recommend first reading the paper "`Integrating Generic Sensor Fusion Algorithms with Sound State Representations through Encapsulation of Manifolds <https://arxiv.org/pdf/1107.1119.pdf>`_" which documents the general ideas utilized and then Jason's "`MIL GNC Notes <https://drive.google.com/open?id=1zyQBhEKoxeEmgvIFYeyVt2m0XVV-EU5y>`_" which describes the specific math that is relevant to MIL robots. The code is a relatively direct implementation of that math, without too much extra boilerplate, mostly thanks to the unscented Kalman filter not requiring the explicit calculation of huge Jacobian matrices, along with some macro metaprogramming that generates manifold operators (which is similar to that described in the first paper, section 5).

There are a few weird/surprising things in the implementation:

Manifold types
--------------

.. note::

   This section is a very concise summary of the key concepts of the the "Sound State Representations" paper linked above.

Central to ``odom_estimator`` is the concept of a "`manifold <https://en.wikipedia.org/wiki/Manifold>`_."

Any manifold space locally looks like a vector space, but globally can have more complicated behavior. The vector space around an infinitesimal point on the manifold is called its "`tangent space <https://en.wikipedia.org/wiki/Tangent_space>`_." This tangent space has a dimension that is a characteristic of the specific manifold that you're dealing with. For example, any point on the manifold of unit quaternions has a tangent space that is three-dimensional. This isn't very surprising, as the `space of unit quaternions or any representation of rotation <https://en.wikipedia.org/wiki/3D_rotation_group>`_ has three degrees of freedom (e.g. roll, pitch, yaw for `Euler angles <https://en.wikipedia.org/wiki/Euler_angles>`_).

Some examples of manifolds with their tangent space dimensions:

======================= =========
Name                    Dimension
======================= =========
an N-dimensional vector N
a unit quaternion       3
a scalar                1
an angle                1
======================= =========

With this in mind, you can define an abstraction over all manifold types that allows you to treat them all generically. For a given manifold:

* You find the dimension of the tangent space of the manifold, ``D``.
* You create a "box-plus" operator that offsets a manifold point along its tangent plane by a D-dimensional vector: ``manifold_point ⊞ x = new_manifold_point``.
* You create a "box-minus" operator that is the inverse of that operation: ``new_manifold_point ⊟ manifold_point = x``.

With these operations, you can write code to generically handle points from any manifold (assuming the points are sufficiently close to each other that the manifold acts roughly linear between them) by just taking an algorithm that works on a vector space (like the Kalman filter) and changing additions/subtractions of values from that vector space into ⊞ and ⊟, and then letting the algorithm operate on vector-valued *perturbations* of the manifold points. In addition, you can handle *distributions* on manifolds by storing the mean as a manifold point along with the covariance as a DxD matrix in the tangent space.

There is a section of macros in ``include/odom_estimator/manifold.h`` that implements a manifold `Cartesian product <https://en.wikipedia.org/wiki/Cartesian_product>`_. You give it several manifold types and it creates a new type that lumps them all together, just like a ``struct``, except that it also generates ``operator+``/``operator-`` that correspond to the box-plus and box-minus operators. With that macro, you can create new manifold types by composing existing ones, and maybe even build up a type that represents the state variable that you're trying to estimate.

These operators allow code like the unscented transform (in ``include/odom_estimator/unscented_transform.h``) and the Kalman filter (in ``include/odom_estimator/kalman.h``) to handle points on manifolds (e.g. a certain unit quaternion in the space of unit quaternions, or an N-dimensional vector, or even both lumped together!) in a completely generic fashion.

ECI coordinates
---------------

Due to ``odom_estimator`` previously supporting fusing raw GPS measurements, the internal state that is estimated is held in `ECI <https://en.wikipedia.org/wiki/Earth-centered_inertial>`_ coordinates. This lets the predict function of the Kalman filter be written without needing to compensate for the `fictitious forces <https://en.wikipedia.org/wiki/Fictitious_force>`_ that arise when working in a rotating reference frame like `ECEF <https://en.wikipedia.org/wiki/ECEF>`_. All this means is that things have to be rotated back/forth pretty often using functions like ``inertial_from_ecef`` and the like. However, said fictitious forces are pretty much negligible; just detecting them with the quality of sensors we have is very difficult. If ``odom_estimator`` were to be rewritten, it would probably use ECEF or ENU coordinates and be clearer as a result.
