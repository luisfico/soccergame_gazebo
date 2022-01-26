/**
 * @author Luis Contreras (educarte.pro@gmail.com)
 * @copyright Copyright (c) 2021
 */

#include <iostream>

//TEMP_19: para controlar con teclas
#include <sys/ioctl.h>
#include <termios.h>


//TEMP_20: sleep en c++ linux
#include <thread>
#include <chrono>

//GZtransport
#include <gazebo/gazebo_config.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>

using namespace std::literals;  //TEMP_20: sleep en c++ linux
  

bool kbhit() //TEMP_19: para controlar con teclas
{
	termios term;
	tcgetattr(0, &term);

	termios term2 = term;
	term2.c_lflag &= ~ICANON;
	tcsetattr(0, TCSANOW, &term2);

	int byteswaiting;
	ioctl(0, FIONREAD, &byteswaiting);

	tcsetattr(0, TCSANOW, &term);

	return byteswaiting > 0;
}

int main(int argc, char** argv)
{

  //GZ --------------------------------------ini
  gazebo::client::setup(argc, argv);

  // Create our node for communication
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();

  // Publish to the  velodyne topic
  gazebo::transport::PublisherPtr pub       =node->Advertise<gazebo::msgs::Vector3d>("/cmdFifaGz");
  gazebo::transport::PublisherPtr pubAction =node->Advertise<gazebo::msgs::Vector3d>("/cmdFifaGzAction");

  //DISABLE WaitForConnection()    because generates problems of init
  //pub->WaitForConnection();         // Wait for a subscriber to connect to this publisher
  //pubAction->WaitForConnection();   // Wait for a subscriber to connect to this publisher
  
  //GZ --------------------------------------end


	FILE *flux = stdin;
	char ch; // ch caracter

	/*
Code ascci teclass
ESC 	27  
SPACE 	32, 
ENTER	??
A  	97
*/
	int contSinPresionar=0;
		
	while (1)
	{	
		double analogValueX=0,analogValueY=0; //PROBLEM se presiona solo una tecla 
	    int buttonR1=0;//Mov: XYaxis , R1: sprint
        int buttonCirculo=0,buttonX=0, buttonCuadrado=0;  //Actions:  circulo:Barrerse,   X:pase,   Cuadrado:patear 
  
		bool isButtonPress=kbhit();
		if (isButtonPress)
		{	contSinPresionar=0;
			ch = getc(flux);
			std::cout << "Apretaste_ " << ch << " _fin" << std::endl;

			//Xaxis	
			//if(ch == 27){ analogValueX=1.0; }
      		//Yaxis
      		//if(ch == 97){ analogValueY=1.0; }//Inverse Y-axis jostick vs screen
      
	        switch (ch)
            {//https://elcodigoascii.com.ar/codigos-ascii/letra-a-minuscula-codigo-ascii-97.html
				//moving
                case 108:               // assci   press l
					std::cout<< "Apretaste tecla l minuscula" <<std::endl;                    
					analogValueX=1.0;
                    	break;
                case 106:               // assci   press j
					std::cout<< "Apretaste tecla j ninuscula" <<std::endl;                    
					analogValueX=-1.0;
	                break;
                case 105:               // assci   press i
					std::cout<< "Apretaste tecla i minuscula" <<std::endl;                    
					analogValueY=1.0;
                    	break;
                case 107:               // assci   press k
					std::cout<< "Apretaste tecla k ninuscula" <<std::endl;                    
					analogValueY=-1.0;
	                break;
				
				//action
				case 122:               // assci   press z  sprint
					std::cout<< "Apretaste tecla z minuscula SPRINT" <<std::endl;                    
					buttonR1=1.0;
                    	break;
                case 115:               // assci   press s  pass
					std::cout<< "Apretaste tecla s minuscula PASS" <<std::endl;                    
					buttonX=1.0;
	                break;
                case 100:               // assci   press d  kick
					std::cout<< "Apretaste tecla d minuscula KICK" <<std::endl;                    
					buttonCuadrado=1.0;
	                break;
				
	         default:
            		std::cout<< "Apretaste otros" <<std::endl;
	     	}
		
		}else{
			contSinPresionar++;
			//std::cout << "contSinPresionar " << contSinPresionar << std::endl;
			if(contSinPresionar==1){
				//std::cout << "NADA \n";
				std::this_thread::sleep_for(300ms);  //TEMP_20: sleep en c++ linux 
			}else{
				std::cout << "SIN PRESIONAR \n";
				std::this_thread::sleep_for(1s);  //TEMP_20: sleep en c++ linux 
				
			}	
			

			//if(3<contSinPresionar &&contSinPresionar<10){
				//std::cout << "SIN PRESIONAR \n";
				//std::this_thread::sleep_for(1s);  //TEMP_20: sleep en c++ linux 
				
			//}else if(contSinPresionar>=10){
			//	contSinPresionar=0;
			//}
			
		}

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

	return (0);
}
