/**
 * @author Luis Contreras (educarte.pro@gmail.com)
 * @copyright Copyright (c) 2021
 */

#include <algorithm>
#include <iostream>
#include <vector>
#include <string>
#include "FBall.h"
#include "FTeam.h"

FBall::FBall(const std::string &_m_name, const std::string &type) : FEntity(_m_name), m_type(type)
{
  std::cout << "Building " << m_name << " level: " << typeid(*this).name() << " \n";
}

FBall::~FBall()
{
  std::cout << "Destroying " << m_name << " level: " << typeid(*this).name() << " \n";
}

std::string FBall::Operation() const
{ // return "FBall";
  return m_name;
}

bool FBall::IsComposite() const
{
  return false;
}

void FBall::showLevel() const
{
  std::cout << "name: " << m_name << ", type: " << m_type << " , level: " << typeid(*this).name() << "\n";
}


void FBall::AssitedPass()
{
FPlayerPtr closestPlayerToBall = this->mPlayers->vecPlayers_BallDistanceSort[0].first;
  
  
      // PASS ASSISTED------------------------ini
      // METHOD FOV()

      // TO USE gz methods for transforms    ejm    file:///home/lc/gzf/gazebo/gazebo/physics/Actor.cc
      // auto matId = ignition::math::Matrix4d::Identity()
      // auto tmp = ignition::math::Matrix4d(animNode->ModelTransform().Rotation()).Inverse();
      // tmp = tmp * ignition::math::Matrix4d(ignition::math::Quaterniond(n.Normalize(), theta));
      // tmp = tmp * ignition::math::Matrix4d(animNode->ModelTransform().Rotation());
      // tmp.SetTranslation(animNode->Transform().Translation());

      double visionRad = 30 * 3.1415 / 180; //+-30°    FOV = vision*2
                                            // Set ball direction during launching by using fov
      //  1) Search the 2nd closest FPlayer in the FOV(field of view) of the FPlayer who has the ball

      // for(auto& elem: this->mPlayers->vecPlayers_BallDistanceSort)
      bool finishSearchPlayerToPass = false;
      // for(int cont=1, cont< mPlayers->vecPlayers_BallDistanceSort.size() && !finishSearchPlayerToPass , cont++ )
      int cont = 1;
      while (cont < this->mPlayers->vecPlayers_BallDistanceSort.size() && !finishSearchPlayerToPass)
      {
        auto closestPlayerToPlayer = this->mPlayers->vecPlayers_BallDistanceSort[cont].first;
        std::cout << "nnnnn cont :  " << cont << std::endl;
        std::cout << "nnnnn Player name :  " << closestPlayerToPlayer->Name() << std::endl;

        // Check if the other player is in the FOV of the main player
        auto tfPlayerWithBall = closestPlayerToBall->mActorGzPtr->WorldPose().Pos();
        auto tfNextPlayer = closestPlayerToPlayer->mActorGzPtr->WorldPose().Pos();

        // Change coordinate system with respect to PlayerWithBall with ball
        auto tfNextPlayer_relativeFrame = (tfNextPlayer - tfPlayerWithBall).Normalize();
        // tfPlayerWithBall.Translation().X()
        // auto YawNextPlayer_relativeFrame = tfNextPlayer_relativeFrame.Rotation().Yaw();  //KO Yaw() incoherent
        auto angZnextPlayer_relativeFrame = atan2(tfNextPlayer_relativeFrame.Y(), tfNextPlayer_relativeFrame.X());
        bool PlayerinFOV = abs(angZnextPlayer_relativeFrame - angZplayer) <= visionRad;

        if (PlayerinFOV)
        { // Set ball direction
          std::cout << "mmmmm PlayerinFOV!!!!!!!!!!!!!!! $$$$$$$$$$$$$$$$$$$$$$$$$ " << std::endl;
          std::cout << "mmmmm fov Player name :  " << closestPlayerToPlayer->Name() << std::endl;
          // angZball=YawNextPlayer_relativeFrame -M_PI_2; //Set ball direction
          angZball = angZnextPlayer_relativeFrame; // Set ball direction
          finishSearchPlayerToPass = true;
        }

        cont++;
      }
      // PASS ASSISTED------------------------end
}

// new__________________
void FBall::Update() // based on show        update keyboard
{
  std::cout << "FBall debe actualizar su pose a partir de su ptr de renderer  \n";
  std::cout << "     Ball World    gravity x " << this->mModelGzPtr->GetWorld()->Gravity().X() << " y " << this->mModelGzPtr->GetWorld()->Gravity().Y() << " z " << this->mModelGzPtr->GetWorld()->Gravity().Z() << std::endl;
  ;
  std::cout << "     Ball World    acceleration x " << this->mModelGzPtr->WorldLinearAccel().X() << " y " << this->mModelGzPtr->WorldLinearAccel().Y() << " z " << this->mModelGzPtr->WorldLinearAccel().Z() << std::endl;
  ;
  std::cout << "     Ball relative acceleration x " << this->mModelGzPtr->RelativeLinearAccel().X() << " y " << this->mModelGzPtr->RelativeLinearAccel().Y() << " z " << this->mModelGzPtr->RelativeLinearAccel().Z() << std::endl;
  ;

  // UpdateState(); // become  update()
  // UpdateRendering();   // become  draw()

  std::cout << "\n Ball:test mPlayers------- : " << this->mPlayers->Name() << " level: " << typeid(*this->mPlayers).name() << "\n";
  // Ball gets the closest player
  FPlayerPtr closestPlayerToBall = this->mPlayers->vecPlayers_BallDistanceSort[0].first;
  double distBallPlayer = this->mPlayers->vecPlayers_BallDistanceSort[0].second;

  std::cout << "Ball: 1st: " << closestPlayerToBall->Name() << " 2nd: " << distBallPlayer << std::endl;

  double distBallPlayerMin = 1; // todo distSmin   eculidiana
  bool ballClosePlayer = (distBallPlayer < distBallPlayerMin);

  // TODO: ERRROR!!!!! el cambio de estado mStateBall==2  a  mStateBall==0
  //   no debe depender del tiempo transcurrido   sino  de la distancia transucurrida!!!!!
  double current_time = this->mModelGzPtr->GetWorld()->SimTime().Double();
  double dTimeInitPass = current_time - mTimeInitPass;
  // Si paso 1.5sec desde que se lanzo el paso  ==> la bola volvera a ser libre:0
  if ((mStateBall == eBallState::INIT_PASS || mStateBall == eBallState::INIT_KICK) && dTimeInitPass > 1.5) // 2:librePorPase(dentro de unos segundos pasara a 0:libre)
  {
    std::cout << "PluginSoccerBall:: mStateBall==2  " << distBallPlayer << std::endl;
    // gazebo::common::Time::Sleep(2); //2seconds
    mStateBall = eBallState::FREE;                                                  // la bola volvera a ser libre:0
    this->mModelGzPtr->GetWorld()->SetGravity(ignition::math::Vector3d(0, 0, -10)); // 9.82 ball left effect
  }
  // Apply ball effect during launch KICK
  if (mStateBall == eBallState::INIT_KICK) // 2:librePorPase(dentro de unos segundos pasara a 0:libre)
  {
    std::cout << "PluginSoccerBall:: EFFECT BALL  " << std::endl;
    // ENABLE TO SEE EFFECT GRAVITY Y= +1 m/s2
    // this->mModelGzPtr->GetWorld()->SetGravity(ignition::math::Vector3d(0, 1, -10)); //9.82 ball left effect

    // TODO apply the effect ball perpendicular to launch direction
    // this->mModelGzPtr->GetWorld()->SetGravity(ignition::math::Vector3d(directionKick_y, directionKick_x, -10)); //9.82 ball left effect
    // ignition::math::Vector3d g = this->mModelGzPtr->GetWorld()->Gravity();
  }

  // Si bola esta cerca y (bola este libre o bola esta controlada)   ==> bola se controlara
  if (ballClosePlayer && (mStateBall == eBallState::FREE || mStateBall == eBallState::CONTROLED)) //==> bola se controlara  mStateBall=1
  {
    mTimeInitPass = 0; // reset mTimeInitPass

    // Change to (ballState: controlada por jugador)
    mStateBall = eBallState::CONTROLED; // controlada
    //closestPlayerToBall->mWithBall = true;

#ifdef DEBUG1 //#endif
    std::cout << "PluginSoccerBall:: Ball must be attached to player!!!   distBallPlayer(m)= " << distBallPlayer << std::endl;
#endif

    // Attaching ball to player:
    double posBallX_framePlayer = 0.5; // La bola esta 1m al frente del jugador respecto al player frame
    // Tranformamos la pose de la bola del playerFrame to worldFrame
    double posXplayer = 1000, posYplayer = 1000;
    posXplayer = closestPlayerToBall->mActorGzPtr->WorldPose().Pos().X();      // using gz interface
    posYplayer = closestPlayerToBall->mActorGzPtr->WorldPose().Pos().Y();      // using gz interface
    angZplayer = closestPlayerToBall->mActorGzPtr->WorldPose().Yaw() - M_PI_2; // using gz interface
    double posBallX_frameWorld = posXplayer + posBallX_framePlayer * cos(angZplayer);
    double posBallY_frameWorld = posYplayer + posBallX_framePlayer * sin(angZplayer);

    // PASS NON ASSISTED
    angZball = angZplayer;


    if (closestPlayerToBall->mStatePlayer == FPlayer::ePlayerState::PASS || closestPlayerToBall->mStatePlayer == FPlayer::ePlayerState::KICK) ////Change to (ballState: libre)       bola lanzada con una velocidad inicial hacia al frente por el momento
    {
    AssitedPass(); //modify   angZball
    }

    // Set ball pose close to link player pose
    // SetBallpose Metodo1: Using Publicar  gazebo/set_model_state     segun  https://www.programmersought.com/article/55451543281/
    // SetBallpose Metodo3: Using SetRelativePose() //Se colocara la bola en un punto de terminado  (ojo hay gravedad)
    ignition::math::Vector3<double> posInit = ignition::math::Vector3d(posBallX_frameWorld, posBallY_frameWorld, 0.2);
    ignition::math::Quaternion<double> rotInit = ignition::math::Quaterniond(1, 0, 0, 0);
    this->mModelGzPtr->SetWorldPose(ignition::math::Pose3d(posInit, rotInit));
    this->mModelGzPtr->SetLinearVel(ignition::math::Vector3d(0, 0, 0)); // TRUCO TMP IMPORTANTE :velocidad de la pelota sera 0 junto al jugador,   para que una vez la pelota sea libre no salga disparada

    // std::cout << " mActionPase= "<<mActionPase<<std::endl;
    if (closestPlayerToBall->mStatePlayer == FPlayer::ePlayerState::PASS) ////Change to (ballState: libre)       bola lanzada con una velocidad inicial hacia al frente por el momento
    {                                                                     // std::cout << "ENTRO!!!!!!!!!!!!!!! $$$$$$$$$$$$$$$$$$$$$$$$$ mActionPase= "<<mActionPase<<std::endl;

      double velInitPase = 6;
      double velInitPaseX = velInitPase * cos(angZball);
      double velInitPaseY = velInitPase * sin(angZball);
      this->mModelGzPtr->SetLinearVel(ignition::math::Vector3d(velInitPaseX, velInitPaseY, 0));
      mStateBall = eBallState::INIT_PASS; // pase
      mTimeInitPass = this->mModelGzPtr->GetWorld()->SimTime().Double();
      // Firstly mActionPase is 1 the action is performed and the actor will change of animation
      // mChangeAnimationActionPass=1;
    }

    // std::cout << " mActionPatear= "<<mActionPatear<<std::endl;
    if (closestPlayerToBall->mStatePlayer == FPlayer::ePlayerState::KICK) ////Change to (ballState: libre)       bola lanzada con una velocidad inicial hacia al frente por el momento
    {                                                                     // std::cout << "ENTRO!!!!!!!!!!!!!!! $$$$$$$$$$$$$$$$$$$$$$$$$ mActionPase= "<<mActionPase<<std::endl;

      double velInitPase = 6;
      double velInitPaseX = velInitPase * cos(angZball);
      double velInitPaseY = velInitPase * sin(angZball);
      this->mModelGzPtr->SetLinearVel(ignition::math::Vector3d(velInitPaseX, velInitPaseY, 5));
      // ball Effect:  Set gravity only during the kick launch

      mStateBall = eBallState::INIT_KICK; // 2:librePorPase(dentro de unos segundos pasara a 0:libre)
      mTimeInitPass = this->mModelGzPtr->GetWorld()->SimTime().Double();
    }
  }
}

void FBall::createBallInRendererGz()
{
  sdf::SDF obj1SDF;
  obj1SDF.SetFromString(models::modelStringFormat_ball); // by default include "libmodel_push1" plugin Unused
  sdf::ElementPtr obj1model = obj1SDF.Root()->GetElement("model");
  obj1model->GetAttribute("name")->SetFromString(this->Name());
  _mWorldPtr->InsertModelSDF(obj1SDF);
}
