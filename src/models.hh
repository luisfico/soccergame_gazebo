/**
 * @author Luis Contreras (educarte.pro@gmail.com)
 * @copyright Copyright (c) 2021
 */

#pragma once

#include <string>
//TODO: to use namespace or enum class
namespace models
{

//sphere with moving plugin  
const std::string modelStringFormat_sphere_withplugin(
    "<sdf version ='1.5'>\
          <model name ='sphere'>\
            <pose>1.5 0 0 0 0 0</pose>\
            <link name ='link'>\
              <pose>0 0 .5 0 0 0</pose>\
              <collision name ='collision'>\
                <geometry>\
                  <sphere><radius>0.5</radius></sphere>\
                </geometry>\
              </collision>\
              <visual name ='visual'>\
                <geometry>\
                  <sphere><radius>0.5</radius></sphere>\
                </geometry>\
              </visual>\
            </link>\
            <plugin name='model_pushX' filename='libmodel_push1.so'/>\
          </model>\
        </sdf>");

const std::string modelStringFormat_sphere(
    "<sdf version ='1.5'>\
          <model name ='sphere'>\
            <pose>1.5 0 0 0 0 0</pose>\
            <link name ='link'>\
              <pose>0 0 .5 0 0 0</pose>\
              <collision name ='collision'>\
                <geometry>\
                  <sphere><radius>0.5</radius></sphere>\
                </geometry>\
              </collision>\
              <visual name ='visual'>\
                <geometry>\
                  <sphere><radius>0.5</radius></sphere>\
                </geometry>\
              </visual>\
            </link>\
          </model>\
        </sdf>");

const std::string modelStringFormat_collisionBox(
    "<sdf version ='1.5'>\    
        <model name='playerX_collision_model'>\
          <pose>0 0 0 0 0 0</pose>\
          <static>true</static>\
          <link name='link'>\
            <collision name='link'>\
              <pose>0 -0.18 0.05 0 -1.5707963267948966 0</pose>\
              <geometry>\
                <box>\
                  <size>0.44 1.62 0.60</size>\
                </box>\
              </geometry>\
            </collision>\
          </link>\
        </model>\
      </sdf>");

//OK player does not need plugins
const std::string  modelStringFormat_player(
  "<sdf version ='1.5'>\
    <actor name='playerX'>\
      <pose>-9.0 0 1.0 1.570796 0.0 3.141593</pose>\
      <skin>\
        <filename>model://playerMov/stop.dae</filename>\
      </skin>\
      <animation name='animation'>\
        <filename>model://playerMov/stop.dae</filename>\
        <interpolate_x>true</interpolate_x>\
      </animation>\
    </actor>\
  </sdf>"); 


const std::string  modelStringFormat_playerB(
  "<sdf version ='1.5'>\
    <actor name='playerX'>\
      <pose>-9.0 0 1.0 1.570796 0.0 3.141593</pose>\
      <skin>\
        <filename>model://actor/meshes/SKIN_man_green_shirt.dae</filename>\
      </skin>\
      <animation name='animation'>\
        <filename>model://actor/meshes/ANIMATION_walking.dae</filename>\
        <interpolate_x>true</interpolate_x>\
      </animation>\
    </actor>\
  </sdf>"); 


  //player with script  KO 
const std::string  modelStringFormat_player_withscript(
  "<sdf version ='1.5'>\
    <actor name='playerX'>\
      <pose>-9.0 0 1.0 1.570796 0.0 3.141593</pose>\
      <skin>\
        <filename>model://playerMov/stop.dae</filename>\
      </skin>\
      <animation name='animation'>\
        <filename>model://playerMov/stop.dae</filename>\
        <interpolate_x>true</interpolate_x>\
      </animation>\
      <script>\
        <trajectory id='0' type='walking'>\
          <waypoint>\
            <time>0</time>\
            <pose>0 2 0 0 0 -1.57</pose>\
          </waypoint>\
          <waypoint>\
            <time>2</time>\
            <pose>0 -2 0 0 0 -1.57</pose>\
          </waypoint>\
        </trajectory>\
      </script>\
    </actor>\
  </sdf>"); 

//player with movement plugin   OK!
const std::string  modelStringFormat_player_withplugin(
  "<sdf version ='1.5'>\
    <actor name='playerX'>\
      <pose>-9.0 0 1.0 1.570796 0.0 3.141593</pose>\
      <skin>\
        <filename>model://playerMov/stop.dae</filename>\
      </skin>\
      <animation name='animation'>\
        <filename>model://playerMov/stop.dae</filename>\
        <interpolate_x>true</interpolate_x>\
      </animation>\
  <plugin name='trajectoryB' filename='libTrajectoryActorPluginB.so'>\
      <target> 12.1254 5.385326 1.0 1.570796 0.0 3.141593</target>\    
      <target>10.3737 3.89311 1.0 1.570796 0.0 3.141593 </target>\
      <target>7.96763 4.71297 1.0 1.570796 0.0 3.141593 </target>\    
      <target>6.41362 5.11439 1.0  1.570796 -0.0 3.141593 </target>\
    <animationb name='animation'>\
        <filename>model://playerMov/run.dae</filename>\
        <interpolate_x>true</interpolate_x>\
    </animationb>\ 
    <animationc name='animation'>\
          <filename>model://playerMov/pass.dae</filename>\ 
          <interpolate_x>false</interpolate_x>\
    </animationc>\ 
    <animationd name='animation'>\
          <filename>model://playerMov/kick.dae</filename>\ 
          <interpolate_x>false</interpolate_x>\
    </animationd>\  
    <velocity>1.06</velocity>\
    <obstacle_margin>1.5</obstacle_margin>\
    <obstacle>my_robot_name</obstacle>\
  </plugin>\
    </actor>\
  </sdf>"); 

const std::string  modelStringFormat_stadiumA(
  "<sdf version ='1.5'>\
    <model name='stadiumA'>\
      <pose>0 0 0 0 0 0</pose>\        
      <include>\
        <uri>model://robocup_3Dsim_field</uri>\
      </include>\
      <include>\
        <uri>model://robocup_3Dsim_goal</uri>\
        <name>robocup_3Dsim_goalB</name>\
        <pose>15 0 0 0 0 0</pose>\
      </include>\
      <include>\
        <uri>model://robocup_3Dsim_goal</uri>\
        <name>robocup_3Dsim_goalA</name>\
        <pose>-15 0 0 0 0 3.1415</pose>\
      </include>\
    </model>\
  </sdf>"); 


const std::string  modelStringFormat_ball(
  "<sdf version ='1.5'>\
    <model name='soccer_ball'>\
      <pose frame=''>-10 -1 0.5 0 0 0</pose>\
      <link name='link'>\
        <collision name='collision'>\
          <geometry>\
            <sphere>\
              <radius>0.1075</radius>\ 
            </sphere>\
          </geometry>\
          <surface>\
            <bounce>\
              <restitution_coefficient>0.5</restitution_coefficient>\
              <threshold>0,1075</threshold>\
            </bounce>\
            <contact>\
              <ode>\
                <max_vel>10</max_vel>\
              </ode>\
            </contact>\
            <friction>\
              <torsional>\
                <ode/>\
              </torsional>\
              <ode/>\
            </friction>\
          </surface>\
          <max_contacts>10</max_contacts>\
        </collision>\
        <inertial>\
          <mass>0.0025</mass>\
          <inertia>\
            <ixx>2.13333e-07</ixx>\
            <ixy>0</ixy>\
            <ixz>0</ixz>\
            <iyy>2.13333e-07</iyy>\
            <iyz>0</iyz>\
            <izz>2.13333e-07</izz>\
          </inertia>\
          <pose frame=''>0 0 0 0 -0 0</pose>\
        </inertial>\
        <visual name='visual'>\
          <geometry>\
            <sphere>\
              <radius>0.1075</radius>\
            </sphere>\
          </geometry>\
          <material>\
            <script>\
              <name>Gazebo/Grey</name>\
              <uri>file://media/materials/scripts/gazebo.material</uri>\
            </script>\
            <ambient>1 1 0 1</ambient>\
            <diffuse>0.7 0.7 0.7 1</diffuse>\
            <specular>0.01 0.01 0.01 1</specular>\
            <emissive>0 0 0 1</emissive>\
            <shader type='vertex'>\
              <normal_map>__default__</normal_map>\
            </shader>\
          </material>\
        </visual>\
        <self_collide>0</self_collide>\
        <enable_wind>0</enable_wind>\
        <kinematic>0</kinematic>\
      </link>\
    </model>\
  </sdf>"); 


}