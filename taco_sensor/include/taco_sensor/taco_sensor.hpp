/**
 * @file   taco_sensor.hpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @date   Thu May 10 16:30:15 2012
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


#ifndef _TACO_SENSOR_HPP_
#define _TACO_SENSOR_HPP_

#include <ros/ros.h>
#include <boost/smart_ptr.hpp>

#include <taco_msgs/TacoReconfigure.h>

namespace taco_sensor
{
  class TacoSensor
  {
  public:
    TacoSensor();
    virtual ~TacoSensor();

    /**
     * The callback for the reconfigure sensor service.
     *
     * @param request the string to send to the sensor to reconfigure it
     * @param response true if success
     *
     * @return true if success
     */
    bool reconfigure_sensor_callback(taco_msgs::TacoReconfigure::Request& request, taco_msgs::TacoReconfigure::Response& response);

  protected:
    ros::NodeHandle node_handle_;

    ///Publishes the foveated data to a point cloud 2
    ros::Publisher foveated_publisher_;
    ///Publishes the unfoveated data to a point cloud 2
    ros::Publisher unfoveated_publisher_;

    ///Publishes the saliency map as an image
    ros::Publisher saliency_map_publisher_;

    ///A service server to reconfigure the sensor on the fly
    ros::ServiceServer reconfigure_sensor_;
  };
} // namespace

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/


#endif
