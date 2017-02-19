This snippet will print out the time that some command takes to execute (plus really little overhead) in C++:

    {
      pcl::ScopeTime t1("calculation");
       centroid_projected_index = sub::project_uv_to_cloud_index(*current_cloud, center, cam_model);
    }

This snippet will do the same in Python:

    from timeit import timeit

    # The first argument is the function to run.
    # The second argument is run before the function and it not included in the execution time.
    # The third argument is the amount of trial runs to average.
    time = timeit("x.index(420)", setup="x = range(1000)", number=1000)