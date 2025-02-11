/**
 * @file TetherPlugin.cc
 * @author Antonio Antunes (antonioantunes5@tecnico.ulisboa.pt)
 * @brief This files implements the TetherPlugin that is loaded in gazebo
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
#include <gazebo/gazebo.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <sdf/sdf.hh>

#include <math.h>

#include <tether_plugin/TetherPlugin.hh>
#include <iostream>

GZ_REGISTER_MODEL_PLUGIN(gazebo::TetherPlugin)

namespace gazebo {

TetherPlugin::TetherPlugin() {}

TetherPlugin::~TetherPlugin() {
  if (this->updateConnection) {
    this->updateConnection.reset();
  }
}

void TetherPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  
  // Store the pointer to the model
  GZ_ASSERT(_model, "TetherPlugin _model pointer is NULL");
  GZ_ASSERT(_sdf, "TetherPlugin _sdf pointer is NULL");

  this->model = _model;
  this->sdf = _sdf;
  this->world = this->model->GetWorld();

  // Read the tether constants from the sdf model
  vehicle_1_name = _sdf->Get<std::string>("vehicle_1");
  vehicle_2_name = _sdf->Get<std::string>("vehicle_2");
  mu = _sdf->Get<double>("mu");
  Lmax = _sdf->Get<double>("Lmax");
  nLinks = _sdf->Get<int>("nLinks");

  // Get the pointers to the links and joints
  for(int i = 0; i < nLinks; i++) {
    link_pointers.push_back(this->model->GetLink(std::string("link_" + std::to_string(i))));
  }

  // Initiate the vector of distances between point links
  distances = std::vector<double>(nLinks);

  // Initialize the visual publisher for updating the cylinders
  node = transport::NodePtr(new transport::Node());
  node->Init();

  pubVisual = node->Advertise<gazebo::msgs::Visual>("~/visual");
  subPose = node->Subscribe("~/pose/info", &TetherPlugin::updatePositionCallback, this);

  drone_position[0] = 0.0;
  drone_position[1] = 15.0;
  drone_position[2] = 35.0;

  ASV_position[0] = -10.0;
  ASV_position[1] = 5.0;
  ASV_position[2] = 0.0;

  poly();

  prevTime = common::Time();

  // Bind the update function to be called by the gazebo periodically
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&TetherPlugin::OnUpdate, this, _1));
}
  
void TetherPlugin::OnUpdate(const common::UpdateInfo &info) {
  //double dt = (double) (info.simTime.sec - prevTime.sec);
  prevTime = info.simTime;

  // Compute the length of the tether
  pos2L(drone_position, ASV_position);

  // Compute the catenary position and force applied on the drone
  catenary(drone_position, ASV_position, prevTime);

  // Update the graphics
  updateGraphics();
}

  //Function that computes the desired length for the tether for a given relative position between vehicles
void TetherPlugin::pos2L(Eigen::Vector3d &pos_drone, Eigen::Vector3d &pos_ASV) {

  Eigen::Vector3d relative_position{0.0, 0.0, 0.0};
  Eigen::Vector4d var{0.0, 0.0, 0.0, 0.0};
  Eigen::Vector3d var_2{0.0, 0.0, 0.0};
  double L1{0};
  double L2{0};
  relative_position = pos_drone - pos_ASV;
  double dist = sqrt(pow(relative_position[0],2) + pow(relative_position[1],2) + pow(relative_position[2],2));
  double x = sqrt(pow(relative_position[0],2) + pow(relative_position[1],2));
  double z = abs(relative_position[2]);
  var[0] = z;
  var[1] = x;
  var[2] = (x*x)/z;
  var[3] = (x*x*x)/(z*z);
  var_2[0] = z;
  var_2[1] = x;
  var_2[2] = (x*x)/z;
  taut = 0;

  if(x/z < infle-0.1)
  {
    Length = p10.dot(var);
  }

  if(x/z >= infle-0.1 && x/z < infle+0.1)
  {
    L1 = p10.dot(var);
    L2 = p10_2.dot(var_2);
    Length = (L1+L2)/2;
  }

  if(x/z >= infle+0.1)
  {
    Length = p10_2.dot(var_2);
  }

  if(atan(z/x) <= 0.45 || atan(z/x) >= 1.3)
  {
    Length = dist + 0.01;
    taut = 1;
  }

  if(Length < dist)
  {
    Length = dist + 0.01;
    taut = 1;
  }

  if(Length > Lmax)
  {
    Length = Lmax;
  }

  if(Length >= Lmax && dist < Lmax) 
  {
    Length = dist + 0.01;
    taut = 1;
  }

}

void TetherPlugin::catenary(Eigen::Vector3d &pos_drone, Eigen::Vector3d &pos_ASV, common::Time &prevTime ) {

  int n = 100, i;
  double coef[n];
  Eigen::Vector3d relative_position{0.0, 0.0, 0.0};
  Eigen::Vector3d D{0.0, 0.0, 0.0};
  Eigen::Vector3d torque{0.0, 0.0, 0.0};
  relative_position = pos_drone - pos_ASV;
  double dist = sqrt(pow(relative_position[0],2) + pow(relative_position[1],2) + pow(relative_position[2],2));
  double x = sqrt(pow(relative_position[0],2) + pow(relative_position[1],2));
  double z = relative_position[2];
  double a = 0.0083;
  double b = 0.1667;
  double c = 1 - ((pow(Length,2)-pow(z,2))/x);
  double r2 = (-b+sqrt(pow(b,2)-4*a*c))/(2*a);
  double H = (mu*x)/(2*sqrt(r2)); //Estimativa inicial
  double v = (mu*x)/(2*H);
  double r = sqrt(pow(Length,2)-pow(z,2))/x;
  double c1 = 0.0,  c2 = 0.0,  f = 0.0,  df = 0.0;
  double theta_uav = 0.0,  theta_base = 0.0, T = 0.0;
  Eigen::Vector2d normP{relative_position[0]/x,relative_position[1]/x}; 

  //Numeric method to compute H, the horizontal reaction force of the cable
  for(i = 0; i < n; i++)
  {
    coef[i] = v;
    f = r-(sinh(v)/v);
    df = -((cosh(v))/v)+((sinh(v))/(pow(v,2)));
    v = v-(f/df);

    if(abs(coef[i]/v) < 1.05 &&  abs(coef[i]/v) > 0.95)
    {
      H = (mu*x)/(2*v);
      break;
    }
  }

  if(H != H)
  {
    H = 0;
  }

  //Compute angles, forces and torques
  a = (2*H/mu);
  r = x/a;
  c1 = asinh((mu*z)/(2*H*sinh(r))) - r;
  c2 = -(H/mu)*cosh(c1);
  theta_uav = atan(sinh((mu*x/H)+c1));
  theta_base = atan(sinh(c1));
  T = H/cos(theta_base);
  D[0] = -H*normP[0];
  D[1] = -H*normP[1];
  D[2] = -H*tan(theta_uav);

  // Get the orientation of the drone so that we can apply the force and torque
  // exerted by the cable on the 3D model
  Eigen::Matrix3d rot = rotationBodyToInertial(drone_attitude_).transpose();
  D = rot * D;
  Eigen::Vector3d point{0.0,0.0,mounting};
  torque[0] = point[1]*D[2]-point[2]*D[1];
  torque[1] = -point[0]*D[2]+point[2]*D[0];
  torque[2] = -point[1]*D[0]+point[0]*D[1];

  if (Length >= Lmax){
    D[0] = 0.0;
    D[1] = 0.0;
    D[2] = 0.0;
    torque[0] = 0.0;
    torque[1] = 0.0;
    torque[2] = 0.0;
  }

  double x_temp[nLinks];
  double step = x/nLinks;
  double Zstep = z/nLinks;
  double z_temp = 0.0;
  double pos[3];

  // Create a Linear Space
  for(int i = 0; i < nLinks; i++) {
    x_temp[i] = x - i*step;
  }
  x_temp[nLinks-1] = 0;

  // Save the vector of positions on the tether
  std::vector<Eigen::Vector3d> positions;

  // Compute the position of the links
  if (taut == 0)
  {
    for(int i = 0; i < nLinks; i++) {
    z_temp = (H/mu)*cosh((mu/H)*x_temp[i]+c1)+c2;
    pos[0] = normP[0]*x_temp[i] + pos_ASV[0];
    pos[1] = normP[1]*x_temp[i] + pos_ASV[1];
    pos[2] = z_temp + pos_ASV[2];
    positions.push_back(Eigen::Vector3d(pos[0], pos[1], pos[2]));
    }
  }

  if (taut == 1)
  {
    for(int i = 0; i < nLinks; i++) {
      z_temp = z - i*Zstep;
      if (i == nLinks-1)
        z_temp = 0;
      pos[0] = normP[0]*x_temp[i] + pos_ASV[0];
      pos[1] = normP[1]*x_temp[i] + pos_ASV[1];
      pos[2] = z_temp + pos_ASV[2];
      positions.push_back(Eigen::Vector3d(pos[0], pos[1], pos[2]));
    }
  }

  // Initialize the link pose
  ignition::math::v6::Pose3<double> link_pose;
  ignition::math::v6::Vector3<double> sc;

  double roll = 0.0;
  double yaw = 0.0;
  double pitch = 0.0;

  // Set the Links in the tether with the correct orientation
  for(int i=1; i < nLinks; i++) {

    // Compute the orientation of the cilinders
    double dx = positions[i][0]-positions[i-1][0];
    double dy = positions[i][1]-positions[i-1][1];
    double dz = positions[i][2]-positions[i-1][2];

    if (i < nLinks) {
      pitch = atan2(-dz,sqrt(dx*dx + dy*dy));
      yaw = atan2(dy, dx);
    } else {
      yaw = 0.0;
      pitch = 0.0;
    }

    double dist = sqrt(pow(dx,2)+pow(dy,2)+pow(dz,2));
    distances[i] = dist;

    // Get the unique visual id for this link (the cilinder)
    uint32_t visual_id;
    link_pointers[i]->VisualId("element_visual", visual_id);

    link_pose = ignition::math::v6::Pose3(positions[i-1][0], positions[i-1][1], positions[i-1][2], roll, pitch, yaw);
    link_pointers[i]->SetWorldPose(link_pose, true, true);
  }

  // Apply the force to the vehicle gazebo model
  gazebo::physics::ModelPtr drone_model = world->ModelByName(vehicle_1_name);
  
  // Try to get the baselink of the model
  gazebo::physics::LinkPtr drone_baselink = drone_model->GetLink("base_link");

  // Some gazebo models use "name/base_link", so try that as well. If not working
  // then throw an std::runtime_error
  if(drone_baselink == nullptr) {
    drone_baselink = drone_model->GetLink(vehicle_1_name + "/base_link");
  }

  // If the "name/base_link" was also not found in the model, the throw the std::runtime_error
  if(drone_baselink == nullptr) {
    throw std::runtime_error("Could not find 'base_link' in " + vehicle_1_name + " gazebo model or '" + vehicle_1_name + "/base_link' in the gazebo model");
  }

  if (z < 1)
  {
    D[0] = 0.0; D[1] = 0.0; D[2] = 0.0;
    torque[0] = 0.0; torque[1] = 0.0; torque[2] = 0.0;
  }

  drone_baselink->SetForce(ignition::math::Vector3d(D[0],D[1],D[2]));
  drone_baselink->SetTorque(ignition::math::Vector3d(torque[0], torque[1], torque[2]));
}

void TetherPlugin::updateGraphics() {

  // For each link, update the cylinder graphics
  for(int i=1; i < nLinks; i++) {

    // Try to get the visual id of the cylinder attached to it
    uint32_t id;
    bool res = link_pointers[i]->VisualId(link_pointers[i]->GetScopedName() + "::element_visual", id);

    // Check the current specs of the cylinder
    msgs::Link msg2;
    link_pointers[i]->FillMsg(msg2);

    for (auto & visualMsg : msg2.visual()) {
      if(visualMsg.has_geometry()) {

        msgs::Geometry geo = visualMsg.geometry();

        // If this is the geometry visual associated with the link i
        if(geo.has_cylinder()) {

          // Set the new geometry equal to the previous one, but with a different length
          msg = visualMsg;
          msg.mutable_geometry()->mutable_cylinder()->set_length(distances[i]);
          msg.mutable_pose()->mutable_position()->set_x(distances[i] * 0.5);
          // Publish the message with the new size of the cylinder
          pubVisual->Publish(msg);
        }
      }
    }
  }
}

void TetherPlugin::updatePositionCallback(const boost::shared_ptr<const gazebo::msgs::PosesStamped> &msg) {

  for (int i = 0; i < msg->pose_size(); i++) {

    // Get the position of the aerial vehicle
    if (std::string(vehicle_1_name).compare(msg->pose(i).name()) == 0) {

      gazebo::msgs::Vector3d uav_pos = msg->pose(i).position();
      gazebo::msgs::Quaternion uav_attitude = msg->pose(i).orientation();

      Eigen::Quaterniond uav_attitude_eigen;
      uav_attitude_eigen.x() = uav_attitude.x();
      uav_attitude_eigen.y() = uav_attitude.y();
      uav_attitude_eigen.z() = uav_attitude.z();
      uav_attitude_eigen.w() = uav_attitude.w();

      drone_attitude_ = quaternion_to_euler(uav_attitude_eigen);

      drone_position[0] = uav_pos.x();
      drone_position[1] = uav_pos.y();
      drone_position[2] = uav_pos.z()+mounting;

    // Get the position of the ASV vehicle
    } else if (std::string(vehicle_2_name).compare(msg->pose(i).name()) == 0) {

      gazebo::msgs::Vector3d asv_pos = msg->pose(i).position();

      ASV_position[0] = asv_pos.x();
      ASV_position[1] = asv_pos.y();
      ASV_position[2] = asv_pos.z() + 0.30;
    }
  }
}

Eigen::Vector3d TetherPlugin::quaternion_to_euler(const Eigen::Quaterniond &q) {
    /* NOTE: The Eigen standard way of doing it is not used because for the order YPR the output range would be:
    [Eigen EulerAngles implementation] yaw, pitch, roll in the ranges [0:pi]x[-pi:pi]x[-pi:pi] */

    Eigen::Vector3d rpy;

    /* Compute roll */
    rpy.x() = std::atan2(2 * (q.w() * q.x() + q.y() * q.z()), 1 - 2 * (q.x() * q.x() + q.y()*q.y()));
    double sin_pitch = 2 * (q.w()*q.y() - q.z()*q.x());
    sin_pitch = sin_pitch >  1 ?  1 : sin_pitch;
    sin_pitch = sin_pitch < -1 ? -1 : sin_pitch;

    /* Compute pitch */
    rpy.y() = std::asin(sin_pitch);

    /* Compute yaw */
    rpy.z() = std::atan2(2 * (q.w() * q.z() + q.x() * q.y()), 1 - 2 * (q.y() * q.y() + q.z() * q.z()));

    return rpy;
}

Eigen::Matrix<double, 3, 3> TetherPlugin::rotationBodyToInertial(const Eigen::Matrix<double, 3, 1> &v) {

    // Create a quaternion
    Eigen::Matrix<double, 3, 3> m;

    // Obtain the orientation according to Z-Y-X convention
    m = (Eigen::AngleAxis<double>(v.z(), Eigen::Matrix<double, 3, 1>::UnitZ()) *
         Eigen::AngleAxis<double>(v.y(), Eigen::Matrix<double, 3, 1>::UnitY()) *
         Eigen::AngleAxis<double>(v.x(), Eigen::Matrix<double, 3, 1>::UnitX())).toRotationMatrix();

    return m;
}

Eigen::Matrix<double, 1, 2> TetherPlugin::cat(Eigen::Vector3d &pos_drone, Eigen::Vector3d &pos_ASV, double L) {
  int n = 100, i;
  double coef[n];
  Eigen::Vector3d relative_position{0.0, 0.0, 0.0};
  relative_position = pos_drone - pos_ASV;
  double dist = sqrt(pow(relative_position[0],2) + pow(relative_position[1],2) + pow(relative_position[2],2));
  double x = sqrt(pow(relative_position[0],2) + pow(relative_position[1],2));
  double z = relative_position[2];
  double a = 0.0083;
  double b = 0.1667;
  double c = 1 - ((pow(L,2)-pow(z,2))/x);
  double r2 = (-b+sqrt(pow(b,2)-4*a*c))/(2*a);
  double H = (mu*x)/(2*sqrt(r2)); //Estimativa inicial
  double v = (mu*x)/(2*H);
  double r = sqrt(pow(L,2)-pow(z,2))/x;
  double c1 = 0.0,  c2 = 0.0,  f = 0.0,  df = 0.0;
  double theta_uav = 0.0,  theta_base = 0.0, T = 0.0;
  Eigen::Vector2d normP{relative_position[0]/x,relative_position[1]/x}; 

  //Numeric method to compute H, the horizontal tension of the cable
  for(i = 0; i < n; i++)
  {
    coef[i] = v;
    f = r-(sinh(v)/v);
    df = -((cosh(v))/v)+((sinh(v))/(pow(v,2)));
    v = v-(f/df);

    if(abs(coef[i]/v) < 1.05 &&  abs(coef[i]/v) > 0.95)
    {
      H = (mu*x)/(2*v);
      break;
    }
  }
  if(H != H)
  {
    H = 0;
  }

  //Compute angles, forces and torques
  a = (2*H/mu);
  r = x/a;
  c1 = asinh((mu*z)/(2*H*sinh(r))) - r;
  c2 = -(H/mu)*cosh(c1);
  theta_base = atan(sinh(c1));
  T = H/cos(theta_base);
  Eigen::Matrix<double, 1, 2> resp{T,theta_base};
  return resp;
}

//Function that computes the polynomial that gives the desired length of the tether 
//for a given relative postion.
void TetherPlugin::poly() {
  double rob = 10;
  double step = 1.03;  //linMassDensity = 30/1000;  miu = norm(g)*linMassDensity; 
  double theta = 999999.0; 
  double L = 0.0;
  double aux ;
  double minim = 999999.0;
  Eigen::Vector3d pos_drone{0.0, 0.0, 0.0};
  Eigen::Vector3d pos_asv{0.0, 0.0, 0.0};
  int num = round(Lmax/step);
  int num2 = 2*num;
  int num3 = num2*num;
  Eigen::MatrixXd slackT(num2,num);
  Eigen::MatrixXd slack(num2,num);
  Eigen::MatrixXd rob90(num2,num);
  Eigen::MatrixXd resp(1,2);

  //Create the discretization along x and z
  Eigen::VectorXd x(num2);
  for (int n = 0; n < num2; ++n)
  {
      x(n) = -Lmax+step*n;
  }
  Eigen::VectorXd z(num);

  for (int n = 0; n < num; ++n)
  {
      z(n) = 0.01+step*n;
  }

  pos_asv[0] = 0.0;
  pos_asv[1] = 0.0;
  pos_asv[2] = 0.0;

  //Compute Slack Tension and its corresponding length
  for(int k = 0; k < num2; ++k)
  {
      for(int j = 0; j < num; ++j)
      {
        pos_drone[0] = 0.0;
        pos_drone[1] = x(k);
        pos_drone[2] = z(j);
        L = sqrt(pow(x(k),2)+pow(z(j),2));
        slackT(k,j) = 0;
        slack(k,j) = 0;
        while(L <= Lmax)
        {
          resp = cat(pos_drone,pos_asv,L);
          theta = resp(1);
          if(abs(theta) <= minim && theta >= 0)
          {
            minim = resp(1);
            slackT(k,j) = resp(0);
            slack(k,j) = L/z(j);
            aux =  slack(k,j);
          }
          L = L + 0.2;
        }
        minim = 999999.0;
      }
  }


  int count = 0;
  Eigen::VectorXd v(num3);
  Eigen::VectorXd t(num3);

  //Compute Length per Unit height for each point in the flying space for a scenario of rob % 
  //greater tension than slack tension
  for(int k = 0; k < num2; ++k)
  {
    for(int j = 0; j < num; ++j)
    {
      L = z(j)*slack(k,j);
      if(L <= Lmax)
      {
        while(L > 0.0)
        {
          pos_drone[0] = 0.0;
          pos_drone[1] = x(k);
          pos_drone[2] = z(j);
          resp = cat(pos_drone,pos_asv,L);
          if(resp(0) >= (1+(rob/100))*slackT(k,j) && resp(0) < (1.1+(rob/100))*slackT(k,j) && resp(1) > 0.0)
          {
            t(count) = abs(x(k)/z(j));
            v(count) = L/z(j);
            rob90(k,j) = L/z(j);
            count++;
            break;
          }
          L = L - 0.2;
        }
      }
    }
  }


  Eigen::MatrixXd A(count,4); 
  Eigen::MatrixXd B(count,3); 
  Eigen::VectorXd vec = v.head(count); 
  Eigen::VectorXd vec2(count); 
  double temp;
  int count2 = 0;

  //Compute the Polynomial Fit via Least Squares for a 3rd degree polynomial
  for(int k = 0; k < count; k++)
  {
    temp = t(k);
    A(k,3) = 1.0; A(k,2) = temp; A(k,1) = pow(temp,2); A(k,0) = pow(temp,3);
  }
  Eigen::MatrixXd coef3 = A.colPivHouseholderQr().solve(vec);

  //Compute the inflection point of the 3rd degree polynomial
  double infle10 = -coef3(1)/(3*coef3(0));

  //Compute the 2nd degree extrapolation to tackle the inflection point consequences
  for(int k = 0; k < count; k++)
  {
    if(t(k) <= infle10)
    {
      vec2(count2) = vec(k);
      temp = t(k);
      B(count2,2) = 1.0; B(count2,1) = temp; B(count2,0) = pow(temp,2); 
      count2++;
    }
  }
  Eigen::VectorXd vec3 = vec2.head(count2); 
  Eigen::MatrixXd BH = B.block(0,0,count2,3); 
  Eigen::MatrixXd coef2 = BH.colPivHouseholderQr().solve(vec3);

  //Set the polynomial coefficients as the output of the function
  Eigen::Matrix<double,1,8> out;
  out(0) = coef3(3); out(1) = coef3(2); out(2) = coef3(1); out(3) = coef3(0);
  out(5) = coef2(2); out(6) = coef2(1); out(7) = coef2(0); out(4) = infle10;
  p10 = out.head(4);
  p10_2 = out.tail(3);
  infle = out(4); 
}

}
