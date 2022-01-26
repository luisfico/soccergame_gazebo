/**
 * @author Luis Contreras (educarte.pro@gmail.com)
 * @copyright Copyright (c) 2021
 */

//USAGE:       $ gz topic --echo /cmdFifaGz


/* Some botons
//---mov
analogo der Xaxis                 mov X
analogo der Yaxis                 mov Y
R1            Button 5 is down    mov sprint
//---actions
triangulo     Button 0 is down    pase largo    
circulo       Button 1 is down    barrese, quitar fuerte
X:            Button 2 is down    pase corto
Cuadrado      Button 3 is down    patear
*/



#include "joystick.hh"
#include <unistd.h>
/*
//ROS :
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
*/
//GZtransport
#include <gazebo/gazebo_config.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>

//#define DEBUG1   // #ifdef DEBUG1   //#endif

int main(int argc, char** argv)
{
/*
  //ROS: Initializes ROS, and sets up a node
  ros::init(argc, argv, "nodePub");
  ros::NodeHandle nh; //manages the node
  //Creates the publisher, and tells it to publish to the  topic, with a queue size of 100
  ros::Publisher pub=nh.advertise<geometry_msgs::Twist>("/cmdFifa", 10);
*/

  //GZ --------------------------------------ini
  gazebo::client::setup(argc, argv);

  // Create our node for communication
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();

  // Publish to the  velodyne topic
  gazebo::transport::PublisherPtr pub       =node->Advertise<gazebo::msgs::Vector3d>("/cmdFifaGz");
  gazebo::transport::PublisherPtr pubAction =node->Advertise<gazebo::msgs::Vector3d>("/cmdFifaGzAction");
  gazebo::transport::PublisherPtr pubTest =node->Advertise<gazebo::msgs::Vector3d>("/cmdTest");

  //DISABLE WaitForConnection()    because generates problems of init
  //pub->WaitForConnection();         // Wait for a subscriber to connect to this publisher
  //pubAction->WaitForConnection();   // Wait for a subscriber to connect to this publisher
  
  //GZ --------------------------------------end

  // Create an instance of Joystick
  Joystick joystick("/dev/input/js0");

  // Ensure that it was found and that we can use it
  if (!joystick.isFound())
  {
    printf("open failed.\n");
    exit(1);
  }

  while (true)
  {
    // Restrict rate
    usleep(1000);

    // Attempt to sample an event from the joystick
    JoystickEvent event;
    if (joystick.sample(&event))
    { 
      double analogValueX,analogValueY; //Be carefull they cannot be 0 !!! 
      int buttonR1=0;//Mov: XYaxis , R1: sprint
      int buttonCirculo=0,buttonX=0, buttonCuadrado=0;  //Actions:  circulo:Barrerse,   X:pase,   Cuadrado:patear 
      
      if (event.isButton())
      {
        #ifdef DEBUG1   //#endif
        printf("Buttonnn %u is %s\n",event.number,event.value == 0 ? "up" : "down");
        #endif
  
        //Mov sprint
        if(event.number == 5){ buttonR1=(int)event.value; }
        //Actions
        if(event.number == 1){ buttonCirculo  =(int)event.value; }
        if(event.number == 2){ buttonX        =(int)event.value; }
        if(event.number == 3){ buttonCuadrado =(int)event.value; }
      }
      else if (event.isAxis())
      {
        #ifdef DEBUG1   //#endif
        printf("Axisss %u is at position %d\n", event.number, event.value);
        #endif

      }

      //Xaxis
      if(event.number == 0){ analogValueX=(double)event.value; }
      //Yaxis
      if(event.number == 1){ analogValueY=-(double)event.value; }//Inverse Y-axis jostick vs screen
      
      /*
      //ROS: Declares the message to be sent
      geometry_msgs::Twist msg;
      //Set message for jostick left analog
      msg.linear.x=analogValueX;
      msg.linear.y=analogValueY;
      //sprint   W in keyboard   , R1 jostick
      msg.linear.z=stateButtonSprint;
      pub.publish(msg);
      */

      //GZ               
      gazebo::msgs::Vector3d msg; // Create a a vector3 message 
      gazebo::msgs::Set(&msg, ignition::math::Vector3d(analogValueX,analogValueY,buttonR1));
      //gazebo::msgs::Set(&msg, ignition::math::Vector3d(1,0,0) );
      pub->Publish(msg); //nodo publicador

      //GZ actions:      circulo:Barrerse,   X:pase,   Cuadrado:patear 
      gazebo::msgs::Vector3d msgAction; // Create a a vector3 message 
      gazebo::msgs::Set(&msgAction, ignition::math::Vector3d(buttonCirculo,buttonX,buttonCuadrado));
      pubAction->Publish(msgAction); //nodo publicador



    }
  }

  gazebo::client::shutdown();
}
