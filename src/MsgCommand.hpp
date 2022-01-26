/**
 * @author Luis Contreras (educarte.pro@gmail.com)
 * @copyright Copyright (c) 2021
 */

#pragma once

struct MsgCommand
{
      //MsgCommand(){};
      //~MsgCommand(){};
      
      //Mode attack
      double movX=0;
      double movY=0;
      double sprint=0;
      double sweep=0; //circle   circle, glob pass
      double pass=0; //x
      double kick=0;  //square   
      //double passLong=0;//triangle 

      //Mode defense
      //double sweep/passGlob=0; //circle   circle, glob pass
      //Symply use *msgCmdLast= *msgCmd
      //           *msgCmd = MsgCommand{mMovX, mMovY, mMovSprint, mActionBarrerse, mActionPase, mActionPatear};
      /*
      void setMsgControlGz(const double mmovX, const double mmovY, const double msprint,
                           const double msweep, const double mpass, const double mkick)
      {
      this->movX = mmovX;
      this->movY = mmovY;
      this->sprint = msprint;
      this->sweep = msweep;
      this->pass = mpass;
      this->kick = mkick;
      }
      */

    //Overloading operator ==
    bool operator == (MsgCommand const &n) {
        return (this->movX == n.movX &&
                this->movY == n.movY &&
                this->kick == n.kick &&
                this->pass == n.pass &&
                this->sprint == n.sprint &&
                this->sweep == n.sweep 
                //this->passLong == n.passLong  
                );
    }



};
using MsgCommandPtr = std::shared_ptr<MsgCommand>;
