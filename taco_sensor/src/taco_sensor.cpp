/**
 * @file   taco_sensor.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @date   Thu May 10 16:31:55 2012
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation, either version 2 of the License, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *
 * @brief A ROS wrapper for the TACO sensor.
 *
 */

#include "taco_sensor/taco_sensor.hpp"

namespace taco_sensor
{
  const unsigned int TacoSensor::taco_width_const_ = 240;
  const unsigned int TacoSensor::taco_height_const_ = 120;

  const std::string TacoSensor::taco_reference_frame_const_ = "taco";

  TacoSensor::TacoSensor()
    : node_handle_("~")
  {
    //initialises our data variables
    foveated_pcl_ = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI> >(new pcl::PointCloud<pcl::PointXYZI>());
    unfoveated_pcl_ = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI> >(new pcl::PointCloud<pcl::PointXYZI>());
    saliency_map_ = boost::shared_ptr<sensor_msgs::Image>(new sensor_msgs::Image());

    init_pcl_(foveated_pcl_);
    init_pcl_(unfoveated_pcl_);
    saliency_map_->width = taco_width_const_;
    saliency_map_->height = taco_height_const_;
    saliency_map_->data.resize( taco_height_const_ * taco_width_const_ );
    saliency_map_->header.frame_id = taco_reference_frame_const_;

    //we're using mono16 to give us some room for labelling etc... (change if needed)
    saliency_map_->encoding = "mono16";

    //initialises the ros publishers and services
    foveated_publisher_ = node_handle_.advertise<pcl::PointCloud<pcl::PointXYZI> >("foveated", 10);
    unfoveated_publisher_ = node_handle_.advertise<pcl::PointCloud<pcl::PointXYZI> >("unfoveated", 10);
    saliency_map_publisher_ = node_handle_.advertise<sensor_msgs::Image>("saliency_map", 10);
    reconfigure_sensor_ = node_handle_.advertiseService("reconfigure_sensor", &TacoSensor::reconfigure_sensor_callback, this);

    /*
     *TODO: I don't think it's working this way around:
     * here I'm using a timer to have a regular callback and to fill in
     * the data structures, but
     * it'll probably be the TACO sensor pushing the data at its own rate
     * in the real implementation.
     */
    data_timer_ = node_handle_.createTimer(ros::Duration(0.05), &TacoSensor::data_timer_callback, this);
  };

  TacoSensor::~TacoSensor()
  {
  };

  void TacoSensor::data_timer_callback(const ros::TimerEvent&)
  {
    foveated_pcl_->header.stamp = ros::Time::now();
    unfoveated_pcl_->header.stamp = ros::Time::now();
    saliency_map_->header.stamp = ros::Time::now();

    //Fill in the different pointers with dummy data
    for(unsigned int i = 0; i < foveated_pcl_->points.size(); ++i)
    {
      foveated_pcl_->points[i].x = static_cast<double>(i) / 100.0;
      foveated_pcl_->points[i].y = static_cast<double>(i) / 100.0;
      foveated_pcl_->points[i].z = static_cast<double>(i) / 100.0;
      foveated_pcl_->points[i].intensity = 1.0;

      unfoveated_pcl_->points[i].x = static_cast<double>(i * i) / 100.0;
      unfoveated_pcl_->points[i].y = static_cast<double>(i * i) / 100.0;
      unfoveated_pcl_->points[i].z = static_cast<double>(i * i) / 100.0;
      unfoveated_pcl_->points[i].intensity = 1.0;
    }

    for(unsigned int i = 0; i < saliency_map_->data.size(); ++i)
    {
      saliency_map_->data[i] = i;
    }

    //publish the data
    publish_all_data_();
  }

  void TacoSensor::publish_all_data_()
  {
    foveated_publisher_.publish( *foveated_pcl_.get() );
    unfoveated_publisher_.publish( *unfoveated_pcl_.get() );
    saliency_map_publisher_.publish( *saliency_map_.get() );
  }

  void TacoSensor::init_pcl_( boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI> > pcl )
  {
    pcl->width = taco_width_const_;
    pcl->height = taco_height_const_;

    pcl->points.resize( taco_width_const_ * taco_height_const_ );

    pcl->header.frame_id = taco_reference_frame_const_;
  }

  bool TacoSensor::reconfigure_sensor_callback(taco_msgs::TacoReconfigure::Request& request, taco_msgs::TacoReconfigure::Response& response)
  {
    ROS_ERROR_STREAM(" Send me to port 12345 for TACO sensor: " << request.configure_command);

    //TODO

    response.success = true;

    return true;
  }

} // namespace

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/

