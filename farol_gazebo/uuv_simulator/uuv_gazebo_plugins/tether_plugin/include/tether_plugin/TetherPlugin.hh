/**
 * @file TetherPlugin.hh
 * @author Antonio Antunes (antonioantunes5@tecnico.ulisboa.pt)
 * @brief This files declares the TetherPlugin class that is loaded in gazebo
 * @version 1.0
 * @date 2022-12-03
 * 
 * @copyright MIT License
 *
 * Copyright (c) 2022 António Antunes
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 *  The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#ifndef __TETHER_PLUGIN_HH__
#define __TETHER_PLUGIN_HH__

#include <vector>
#include <string>
#include <fstream>
#include <iostream>
#include <algorithm>
#include <Eigen/Dense>
#include <ignition/math.hh>

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/msgs/msgs.hh>
#include <boost/shared_ptr.hpp>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Assert.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/transport/TransportTypes.hh>

#include <sdf/sdf.hh>

namespace gazebo {

// \brief Class for the thruster plugin
class TetherPlugin : public ModelPlugin {
  
public: 
  
  // Constructor
  TetherPlugin();

  // Destructor
  virtual ~TetherPlugin();

  // Load the plugin model part
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

  // Update the simulation state. Receives the information used in the update event.
  void OnUpdate(const common::UpdateInfo &_info);
  
protected: 

  void poly();

  void pos2L(Eigen::Vector3d &pos_drone, Eigen::Vector3d &pos_ASV);

	void catenary(Eigen::Vector3d &pos_drone, Eigen::Vector3d &pos_ASV, common::Time &prevTime);

  void updatePositionCallback(const boost::shared_ptr<const gazebo::msgs::PosesStamped> &msg);

	void updateGraphics();
  
  // Update event
  event::ConnectionPtr updateConnection;

  // Gazebo node
  protected: transport::NodePtr node;  

  Eigen::Vector3d quaternion_to_euler(const Eigen::Quaterniond &q);
	Eigen::Matrix<double, 3, 3> rotationBodyToInertial(const Eigen::Matrix<double, 3, 1> &v);
	Eigen::Matrix<double, 1, 2> cat(Eigen::Vector3d &pos_drone, Eigen::Vector3d &pos_ASV, double L); 

  double Length{50}; //Maximum tether length
  int nLinks{0};
  int taut = 0;
  double l = 0.0;
  double h = 0.0;
  double mu = 0.0;
  double Lmax = 0.0;
  double infle = 0.0;
  double mounting = -0.1;
  Eigen::Vector4d p10{0.9703,0.2012,0.3331,-0.0533};
  Eigen::Vector3d p10_2{0.9582,0.2907,0.1990};

  /// \brief Vehicle model names in gazebo to which the tether will be attached to 
  std::string vehicle_1_name{""};
  std::string vehicle_2_name{""};
  
  /// \brief the current drone position
  Eigen::Vector3d drone_position{0.0, 0.0, 0.0};
  Eigen::Vector3d ASV_position{0.0, 0.0, 0.0};

  // Vector of link pointers and joint pointers
  std::vector<boost::shared_ptr<gazebo::physics::Link>> link_pointers;

  // Vector of distances between the points that define the catenary
  std::vector<double> distances;
  
  /// \brief Pointer to the drone base_link
  physics::LinkPtr link_base_drone;

  /// \brief SDF for this plugin;
  sdf::ElementPtr sdf;
  Eigen::Vector3d drone_attitude_;
  // Pointer to the model
  physics::ModelPtr model;
  // Pointer to the current world
  physics::WorldPtr world;
  // Pointer to the update event connection
  msgs::Visual msg;
  // The pointer to publisher to send a command to update a visual.
  transport::PublisherPtr pubVisual;
  // The pointer to a subscribers of the drone and ASV positions
  transport::SubscriberPtr subPose;
  // The pointer to node for communication.
  common::Time prevTime;

};

}

#endif