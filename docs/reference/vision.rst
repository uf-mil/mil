Computer Vision
---------------

Utility Functions
^^^^^^^^^^^^^^^^^
.. autofunction:: mil_vision_tools.auto_canny

.. autofunction:: mil_vision_tools.contour_centroid

.. autofunction:: mil_vision_tools.contour_mask

.. autofunction:: mil_vision_tools.putText_ul

.. autofunction:: mil_vision_tools.points_in_image

.. autofunction:: mil_vision_tools.roi_enclosing_points

.. autofunction:: mil_vision_tools.rect_from_roi

.. autofunction:: mil_vision_tools.quaternion_from_rvec

.. autofunction:: mil_vision_tools.create_object_msg

.. TODO Figure out why this signature is causing issues with Sphinx/Breathe/Doxygen
.. .. doxygenfunction:: mil_vision::pseudoInverse

.. doxygenfunction:: mil_vision::larger_contour

.. doxygenfunction:: mil_vision::smooth_histogram

.. doxygenfunction:: mil_vision::generate_gaussian_kernel_1D

.. doxygenfunction:: mil_vision::find_local_maxima

.. doxygenfunction:: mil_vision::find_local_minima

.. doxygenfunction:: mil_vision::select_hist_mode (std::vector< cv::Point > &histogram_modes, unsigned int target)

.. doxygenfunction:: mil_vision::select_hist_mode (std::vector< cv::Point > &histogram_modes, int target)

.. doxygenfunction:: mil_vision::range_from_param

.. doxygenfunction:: mil_vision::inParamRange

.. doxygenfunction:: mil_vision::rotateKernel

.. doxygenfunction:: mil_vision::makeRotInvariant

.. doxygenfunction:: mil_vision::getRadialSymmetryAngle

C++ Type Aliases
^^^^^^^^^^^^^^^^
.. doxygentypedef:: mil_vision::PCD

.. doxygentypedef:: mil_vision::PCDPtr

.. doxygentypedef:: mil_vision::SPtrVector

.. doxygentypedef:: mil_vision::UPtrVector

Vision Utility Functions
^^^^^^^^^^^^^^^^^^^^^^^^
.. doxygenfunction:: mil_vision::kanatani_triangulation

.. doxygenfunction:: mil_vision::statistical_image_segmentation

.. doxygenfunction:: mil_vision::triangulate_Linear_LS

.. doxygenfunction:: mil_vision::lindstrom_triangulation

ContourClassifier
^^^^^^^^^^^^^^^^^
.. attributetable:: mil_vision_tools.ContourClassifier

.. autoclass:: mil_vision_tools.ContourClassifier
    :members:

Threshold
^^^^^^^^^
.. attributetable:: mil_vision_tools.Threshold

.. autoclass:: mil_vision_tools.Threshold
    :members:

ImageMux
^^^^^^^^
.. attributetable:: mil_vision_tools.ImageMux

.. autoclass:: mil_vision_tools.ImageMux
    :members:

ImageSet
^^^^^^^^
.. attributetable:: mil_vision_tools.ImageSet

.. autoclass:: mil_vision_tools.ImageSet
    :members:

ImageProc
^^^^^^^^^
.. attributetable:: mil_vision_tools.ImageProc

.. autoclass:: mil_vision_tools.ImageProc
    :members:

TrackedObject
^^^^^^^^^^^^^
.. attributetable:: mil_vision_tools.TrackedObject

.. autoclass:: mil_vision_tools.TrackedObject
    :members:

ObjectsTracker
^^^^^^^^^^^^^^
.. attributetable:: mil_vision_tools.ObjectsTracker

.. autoclass:: mil_vision_tools.ObjectsTracker
    :members:

RectFinder
^^^^^^^^^^
.. attributetable:: mil_vision_tools.RectFinder

.. autoclass:: mil_vision_tools.RectFinder
    :members:

VisionNode
^^^^^^^^^^
.. attributetable:: mil_vision_tools.VisionNode

.. autoclass:: mil_vision_tools.VisionNode
    :members:

CameraLidarTransformer
^^^^^^^^^^^^^^^^^^^^^^
.. cppattributetable:: CameraLidarTransformer

.. note::

   Most of the methods/attributes for this class are private, which is why none appear.
   To have these documented, either make these public or discuss the option of documenting
   private C++ class members with a software lead.

.. doxygenclass:: CameraLidarTransformer

ClosedCurve
^^^^^^^^^^^
.. cppattributetable:: mil_vision::ClosedCurve

.. doxygenclass:: mil_vision::ClosedCurve

ActiveContour
^^^^^^^^^^^^^
.. cppattributetable:: mil_vision::ActiveContour

.. doxygenclass:: mil_vision::ActiveContour

CameraObserver
^^^^^^^^^^^^^^
.. cppattributetable:: mil_vision::CameraObserver

.. doxygenclass:: mil_vision::CameraObserver

ColorObservation
^^^^^^^^^^^^^^^^
.. cppattributetable:: mil_vision::ColorObservation

.. doxygenstruct:: mil_vision::ColorObservation

UnoccludedPointsImg
^^^^^^^^^^^^^^^^^^^
.. cppattributetable:: mil_vision::UnoccludedPointsImg

.. doxygenclass:: mil_vision::UnoccludedPointsImg

PointColorStats
^^^^^^^^^^^^^^^
.. cppattributetable:: mil_vision::PointColorStats

.. doxygenstruct:: mil_vision::PointColorStats

PcdColorizer
^^^^^^^^^^^^
.. cppattributetable:: mil_vision::PcdColorizer

.. doxygenclass:: mil_vision::PcdColorizer

SingleCloudProcessor
^^^^^^^^^^^^^^^^^^^^
.. cppattributetable:: mil_vision::SingleCloudProcessor

.. doxygenclass:: mil_vision::SingleCloudProcessor

PixelType
^^^^^^^^^
.. doxygenenum:: mil_vision::PixelType

CameraFrame
^^^^^^^^^^^
.. cppattributetable:: mil_vision::CameraFrame

.. doxygenclass:: mil_vision::CameraFrame

ImageWithCameraInfo
^^^^^^^^^^^^^^^^^^^
.. cppattributetable:: mil_vision::ImageWithCameraInfo

.. doxygenstruct:: mil_vision::ImageWithCameraInfo

FrameHistory
^^^^^^^^^^^^
.. cppattributetable:: mil_vision::FrameHistory

.. doxygenclass:: mil_vision::FrameHistory

Range
^^^^^
.. cppattributetable:: mil_vision::Range

.. doxygenstruct:: mil_vision::Range

CameraFrameSequence
^^^^^^^^^^^^^^^^^^^
.. cppattributetable:: mil_vision::CameraFrameSequence

.. doxygenclass:: mil_vision::CameraFrameSequence

CameraModel
^^^^^^^^^^^
.. cppattributetable:: mil_vision::CameraModel

.. doxygenclass:: mil_vision::CameraModel

ROSCameraStream
^^^^^^^^^^^^^^^
.. cppattributetable:: mil_vision::ROSCameraStream

.. doxygenclass:: mil_vision::ROSCameraStream

PcdSubPubAlgorithm
^^^^^^^^^^^^^^^^^^
.. cppattributetable:: mil_vision::PcdSubPubAlgorithm

.. doxygenclass:: mil_vision::PcdSubPubAlgorithm


ImagePublisher
^^^^^^^^^^^^^^^^^^
.. cppattributetable:: ImagePublisher

.. doxygenclass:: ImagePublisher


ImageSubscriber
^^^^^^^^^^^^^^^^^^
.. cppattributetable:: ImageSubscriber

.. doxygenclass:: ImageSubscriber
