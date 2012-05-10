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

#include <boost/smart_ptr.hpp>

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "taco_sensor");

  boost::shared_ptr<taco_sensor::TacoSensor> taco_sensor = boost::shared_ptr<taco_sensor::TacoSensor>(new taco_sensor::TacoSensor());

  ros::spin();

  return 0;
}


/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/

