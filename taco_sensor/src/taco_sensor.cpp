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

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>

namespace taco_sensor
{
  TacoSensor::TacoSensor()
    : node_handle_("~")
  {
    foveated_publisher_ = node_handle_.advertise<sensor_msgs::PointCloud2>("foveated", 10);

    unfoveated_publisher_ = node_handle_.advertise<sensor_msgs::PointCloud2>("unfoveated", 10);

    saliency_map_publisher_ = node_handle_.advertise<sensor_msgs::Image>("saliency_map", 10);

    reconfigure_sensor_ = node_handle_.advertiseService("reconfigure_sensor", &TacoSensor::reconfigure_sensor_callback, this);
  };

  TacoSensor::~TacoSensor()
  {

  };

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

