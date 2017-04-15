#include <blueview_ros_driver.hpp>

BlueViewRosDriver::BlueViewRosDriver() : nh(ros::this_node::getName()), image_transport(nh)
{
  initParams();
}
void BlueViewRosDriver::initParams()
{
  // Get sonar frame_id for header
  nh.param<std::string>("sonar_frame", frame_id, "blueview");

  // Determine which topics to publish
  nh.param<bool>("grayscale/enable", do_grayscale, false);
  nh.param<bool>("color/enable", do_color, false);
  nh.param<bool>("raw/enable", do_raw, true);
  if (do_grayscale)
  {
    grayscale_img.reset(new cv_bridge::CvImage());
    grayscale_img->encoding = "mono16";
    grayscale_img->header.frame_id = frame_id;
    grayscale_pub = image_transport.advertise("image_mono", 1);
  }
  if (do_color)
  {
    // Get and load color map
    std::string color_map_file;
    if (nh.getParam("color/map_file", color_map_file))
      sonar.loadColorMapper(color_map_file);

    color_img.reset(new cv_bridge::CvImage());
    color_img->encoding = "bgra8";
    color_img->header.frame_id = frame_id;
    color_pub = image_transport.advertise("image_color", 1);
  }
  if (do_raw)
  {
    ping_msg.reset(new mil_blueview_driver::BlueViewPing());
    ping_msg->header.frame_id = frame_id;
    raw_pub = nh.advertise<mil_blueview_driver::BlueViewPing>("ranges", 5);
  }
  std::string params;
  if (nh.getParam("file", params))
    sonar.init(BlueViewSonar::ConnectionType::FILE, params);
  else if (nh.getParam("device", params))
    sonar.init(BlueViewSonar::ConnectionType::DEVICE, params);
  else
    throw std::runtime_error("Can not connect: neither 'file' or 'device' param set");

  // Set Ranges, meters
  BVTSDK::Head &head = sonar.getHead();
  float range_lower, range_upper;
  if (nh.getParam("range/start", range_lower))
  {
    head.SetStartRange(range_lower);
  }
  if (nh.getParam("range/stop", range_upper))
  {
    head.SetStopRange(range_upper);
  }

  // Handle fluid type, enum string
  std::string fluid_type;
  if (nh.getParam("fluid_type", fluid_type))
  {
    if (fluid_type == "saltwater")
      head.SetFluidType(BVTSDK::FluidType::Saltwater);
    else if (fluid_type == "freshwater")
      head.SetFluidType(BVTSDK::FluidType::Freshwater);
    else if (fluid_type == "other")
      head.SetFluidType(BVTSDK::FluidType::Other);
    else
      ROS_ERROR("Fluid type (%s) invalid, must be 'saltwater', 'freshwater', "
                "or 'other'",
                fluid_type.c_str());
  }

  // Set sound speed, m/s
  int sound_speed;
  if (nh.getParam("sound_speed", sound_speed))
    head.SetSoundSpeed(sound_speed);

  float range_resolution;
  if (nh.getParam("range_resolution", range_resolution))
    head.SetRangeResolution(range_resolution);

  // Set analog gain adjustment in dB
  float gain;
  if (nh.getParam("gain_adjustment", gain))
    head.SetGainAdjustment(gain);

  // Set "time variable analog gain", in dB/meter,
  float tvg;
  if (nh.getParam("tvg_slope", tvg))
    head.SetTVGSlope(tvg);

  // Set dynamic power managment
  bool dynamic_power;
  if (nh.getParam("dynamic_power_management", dynamic_power))
    head.SetDynamicPowerManagement(dynamic_power);

  // Set ping interval
  float ping_interval;
  if (nh.getParam("ping_interval", ping_interval))
    head.SetPingInterval(ping_interval);
  sonar.updateHead();

  int range_profile_thresh;
  if(nh.param<int>("range_profile_intensity_threshold", range_profile_thresh))
    sonar.SetRangeProfileMinIntensity(range_profile_thresh);

  float noise_threshold;
  if(nh.param<int>("noise_threshold", noise_threshold))
    sonar.SetRangeProfileMinIntensity(noise_threshold);

  // Start loop
  nh.param<double>("period_seconds", period_seconds, -1);
}
void BlueViewRosDriver::run()
{
    if (period_seconds <= 0.0)
    {
      while (ros::ok())
      {
        get_ping();
        ros::spinOnce();
      }
    }
    else {
      timer = nh.createTimer(ros::Duration(period_seconds), std::bind(&BlueViewRosDriver::loop, this, std::placeholders::_1));
      ros::spin();
    }
}
void BlueViewRosDriver::get_ping()
{
  if (!sonar.getNextPing())
  {
    ROS_WARN("No pings remaining in file, shutting down...");
    ros::shutdown();
    return;
  }
  // Do images
  if (do_grayscale || do_color)
  {
    sonar.generateImage();
    if (do_grayscale)
    {
      sonar.getGrayscaleImage(grayscale_img->image);
      grayscale_pub.publish(grayscale_img->toImageMsg());
    }
    if (do_color)
    {
      sonar.getColorImage(color_img->image);
      color_pub.publish(color_img->toImageMsg());
    }
  }
  if (do_raw)
  {
    ping_msg->header.stamp = ros::Time::now();
    sonar.getRanges(ping_msg->bearings, ping_msg->ranges, ping_msg->intensities);
    raw_pub.publish(ping_msg);
  }
}
void BlueViewRosDriver::loop(const ros::TimerEvent &)
{
  get_ping();
}

int main(int argc, char **argv)
{
  try
  {
    ros::init(argc, argv, "blue_view_driver");
    BlueViewRosDriver driver;
    driver.run();
  }
  catch (const BVTSDK::SdkException &err)
  {
    ROS_FATAL("Exception thrown in Blue View SDK (Error #%d):\n\t%s: %s\n", err.ReturnCode(), err.ErrorName().c_str(),
           err.ErrorMessage().c_str());
  } 
  catch (const std::runtime_error& err) 
  {
    ROS_FATAL("Exception: %s", err.what());
  }
}
