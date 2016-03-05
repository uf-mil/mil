

# Scope time!
This snippet will print out the time that some command takes to execute (plus really little overhead)


    {
      pcl::ScopeTime t1("calculation");
      centroid_projected_index = sub::project_uv_to_cloud_index(*current_cloud, center, cam_model);
    }
