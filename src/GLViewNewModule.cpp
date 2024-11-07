#include "GLViewNewModule.h"

#include "WorldList.h" //This is where we place all of our WOs
#include "ManagerOpenGLState.h" //We can change OpenGL State attributes with this
#include "Axes.h" //We can set Axes to on/off with this
#include "PhysicsEngineODE.h"

//Different WO used by this module
#include "WO.h"
#include "WOStatic.h"
#include "WOStaticPlane.h"
#include "WOStaticTrimesh.h"
#include "WOTrimesh.h"
#include "WOHumanCyborg.h"
#include "WOHumanCal3DPaladin.h"
#include "WOWayPointSpherical.h"
#include "WOLight.h"
#include "WOSkyBox.h"
#include "WOCar1970sBeater.h"
#include "Camera.h"
#include "CameraStandard.h"
#include "CameraChaseActorSmooth.h"
#include "CameraChaseActorAbsNormal.h"
#include "CameraChaseActorRelNormal.h"
#include "Model.h"
#include "ModelDataShared.h"
#include "ModelMesh.h"
#include "ModelMeshDataShared.h"
#include "ModelMeshSkin.h"
#include "WONVStaticPlane.h"
#include "WONVPhysX.h"
#include "WONVDynSphere.h"
#include "WOImGui.h" //GUI Demos also need to #include "AftrImGuiIncludes.h"
#include "AftrImGuiIncludes.h"
#include "AftrGLRendererBase.h"

#include "MGLAxes.h"
#include "IndexedGeometryTriangles.h"
#include "vtkOFRenderer.hpp"


using namespace Aftr;

static vtkOFRenderer openFoamRenderer("C:/Users/wellm/Projects/openFoam/pitzDailySteady/");

GLViewNewModule* GLViewNewModule::New( const std::vector< std::string >& args )
{
   GLViewNewModule* glv = new GLViewNewModule( args );
   glv->init( Aftr::GRAVITY, Vector( 0, 0, -1.0f ), "aftr.conf", PHYSICS_ENGINE_TYPE::petODE );
   glv->onCreate();
   return glv;
}


GLViewNewModule::GLViewNewModule( const std::vector< std::string >& args ) : GLView( args )
{
   //Initialize any member variables that need to be used inside of LoadMap() here.
   //Note: At this point, the Managers are not yet initialized. The Engine initialization
   //occurs immediately after this method returns (see GLViewNewModule::New() for
   //reference). Then the engine invoke's GLView::loadMap() for this module.
   //After loadMap() returns, GLView::onCreate is finally invoked.

   //The order of execution of a module startup:
   //GLView::New() is invoked:
   //    calls GLView::init()
   //       calls GLView::loadMap() (as well as initializing the engine's Managers)
   //    calls GLView::onCreate()

   //GLViewNewModule::onCreate() is invoked after this module's LoadMap() is completed.
}


void GLViewNewModule::onCreate()
{
   //GLViewNewModule::onCreate() is invoked after this module's LoadMap() is completed.
   //At this point, all the managers are initialized. That is, the engine is fully initialized.

   if( this->pe != NULL )
   {
      //optionally, change gravity direction and magnitude here
      //The user could load these values from the module's aftr.conf
      this->pe->setGravityNormalizedVector( Vector( 0,0,-1.0f ) );
      this->pe->setGravityScalar( Aftr::GRAVITY );
   }
   this->setActorChaseType( STANDARDEZNAV ); //Default is STANDARDEZNAV mode
   //this->setNumPhysicsStepsPerRender( 0 ); //pause physics engine on start up; will remain paused till set to 1
}


GLViewNewModule::~GLViewNewModule()
{
   //Implicitly calls GLView::~GLView()
}


void updateSpitball(WO* wo) {
    static double velocity = 0.1f;
    const float grav = 0.0f;
    static float time = 0;
    Vector pos = wo->getPosition();
    wo->setPosition(Vector(pos.x, pos.y + cos(time), pos.z + sin(time)));
    time+=0.2;
}

void GLViewNewModule::updateWorld()
{
   GLView::updateWorld(); //Just call the parent's update world first.
                          //If you want to add additional functionality, do it after
                          //this call.
   
   updateSpitball(this->getActor());

   WorldContainer* wl = this->getWorldContainer();
   //std::cout << wl->size() << std::endl;
   openFoamRenderer.updateVtkTrackModel(wl);
}


void GLViewNewModule::onResizeWindow( GLsizei width, GLsizei height )
{
   GLView::onResizeWindow( width, height ); //call parent's resize method.
}


void GLViewNewModule::onMouseDown( const SDL_MouseButtonEvent& e )
{
   GLView::onMouseDown( e );
}


void GLViewNewModule::onMouseUp( const SDL_MouseButtonEvent& e )
{
   GLView::onMouseUp( e );
}


void GLViewNewModule::onMouseMove( const SDL_MouseMotionEvent& e )
{
   GLView::onMouseMove( e );
}


void GLViewNewModule::onKeyDown( const SDL_KeyboardEvent& key )
{

   GLView::onKeyDown( key );
   if( key.keysym.sym == SDLK_0 )
      this->setNumPhysicsStepsPerRender( 1 );

   if( key.keysym.sym == SDLK_1 )
   {

   }

   // movement keys
   if (key.keysym.sym == SDLK_w) {
       this->cam->moveInLookDirection();
   } else if (key.keysym.sym == SDLK_a) {
       this->cam->moveLeft();
   } else if (key.keysym.sym == SDLK_d) {
       this->cam->moveRight();
   } else if (key.keysym.sym == SDLK_s) {
       this->cam->moveOppositeLookDirection();
   }

   // change FOV with - and =
   if (key.keysym.sym == SDLK_MINUS) {
       this->cam->setCameraVerticalFOV(this->cam->getCameraVerticalFOVDeg() - 5);
       this->cam->setCameraHorizontalFOV(this->cam->getCameraHorizontalFOVDeg() - 5);
   } else if (key.keysym.sym == SDLK_EQUALS) {
       this->cam->setCameraVerticalFOV(this->cam->getCameraVerticalFOVDeg() + 5);
       this->cam->setCameraHorizontalFOV(this->cam->getCameraHorizontalFOVDeg() + 5);
   }

   if (key.keysym.sym == SDLK_SPACE) {

   }
}


void GLViewNewModule::onKeyUp( const SDL_KeyboardEvent& key )
{
   GLView::onKeyUp( key );
}


void initVtkRenderer(vtkOFRenderer* renderer) {
    renderer->parseTracksFiles();
}

void Aftr::GLViewNewModule::loadMap()
{
   
   // open foam case directory (obviously this is different between computers)
   /*OpenFOAM parser/renderer initialization!*/
   //openFoamRenderer = vtkOFRenderer("C:/Users/wellm/Projects/openFoam/pitzDailySteady/");
   if(!openFoamRenderer.isReady) 
       initVtkRenderer(&openFoamRenderer);


   this->worldLst = new WorldList(); //WorldList is a 'smart' vector that is used to store WO*'s
   this->actorLst = new WorldList();
   this->netLst = new WorldList();

   ManagerOpenGLState::GL_CLIPPING_PLANE = 1000.0;
   ManagerOpenGLState::GL_NEAR_PLANE = 0.1f;
   ManagerOpenGLState::enableFrustumCulling = false;
   Axes::isVisible = true;
   this->glRenderer->isUsingShadowMapping( false ); //set to TRUE to enable shadow mapping, must be using GL 3.2+
   this->cam->setPosition( 15,15,10 );

   std::string shinyRedPlasticCube( ManagerEnvironmentConfiguration::getSMM() + "/models/cube4x4x4redShinyPlastic_pp.wrl" );
   std::string wheeledCar( ManagerEnvironmentConfiguration::getSMM() + "/models/rcx_treads.wrl" );
   std::string grass( ManagerEnvironmentConfiguration::getSMM() + "/models/grassFloor400x400_pp.wrl" );
   std::string human( ManagerEnvironmentConfiguration::getSMM() + "/models/human_chest.wrl" );
   
   //SkyBox Textures readily available
   std::vector< std::string > skyBoxImageNames; //vector to store texture paths

   skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/sky_deepsun+6.jpg" );


   
      //Create a light
      float ga = 0.1f; //Global Ambient Light level for this module
      ManagerLight::setGlobalAmbientLight( aftrColor4f( ga, ga, ga, 1.0f ) );
      WOLight* light = WOLight::New();
      light->setColor(aftrColor4f(1.0f, 1.0f, 1.0f, 1.0f));
      light->isDirectionalLight( true );
      light->setPosition( Vector( 0, 0, 100 ) );
      //Set the light's display matrix such that it casts light in a direction parallel to the -z axis (ie, downwards as though it was "high noon")
      //for shadow mapping to work, this->glRenderer->isUsingShadowMapping( true ), must be invoked.
      light->getModel()->setDisplayMatrix( Mat4::rotateIdentityMat( { 0, 1, 0 }, 90.0f * Aftr::DEGtoRAD ) );
      light->setLabel( "Light" );
      worldLst->push_back( light );

   
   {
      //Create the SkyBox
      WO* wo = WOSkyBox::New( skyBoxImageNames.at( 0 ), this->getCameraPtrPtr() );
      wo->setPosition( Vector( 0, 0, 0 ) );
      wo->setLabel( "Sky Box" );
      wo->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
      worldLst->push_back( wo );
   }

   { 
      ////Create the infinite grass plane (the floor)
     /* WO* wo = WO::New(grass, Vector(1, 1, 1), MESH_SHADING_TYPE::mstFLAT);
      wo->setPosition( Vector( 0, 0, 0 ) );
      wo->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
      wo->upon_async_model_loaded( [wo]()
         {
            ModelMeshSkin& grassSkin = wo->getModel()->getModelDataShared()->getModelMeshes().at( 0 )->getSkins().at( 0 );
            grassSkin.getMultiTextureSet().at( 0 ).setTexRepeats( 5.0f );
            grassSkin.setAmbient( aftrColor4f( 0.4f, 0.4f, 0.4f, 1.0f ) ); //Color of object when it is not in any light
            grassSkin.setDiffuse( aftrColor4f( 1.0f, 1.0f, 1.0f, 1.0f ) ); //Diffuse color components (ie, matte shading color of this object)
            grassSkin.setSpecular( aftrColor4f( 0.4f, 0.4f, 0.4f, 1.0f ) ); //Specular color component (ie, how "shiney" it is)
            grassSkin.setSpecularCoefficient( 10 ); // How "sharp" are the specular highlights (bigger is sharper, 1000 is very sharp, 10 is very dull)
         } );
      wo->setLabel( "Grass" );
      worldLst->push_back( wo );*/
   }

   

   /*
   *    Render OpenFOAM vtk file. 
   *    
   */
   {
   
       WO* ofobj = openFoamRenderer.renderTimeStampTrack(this->worldLst);
       //worldLst->push_back(ofobj);

   }



   //Make a Dear Im Gui instance via the WOImGui in the engine... This calls
   //the default Dear ImGui demo that shows all the features... To create your own,
   //inherit from WOImGui and override WOImGui::drawImGui_for_this_frame(...) (among any others you need).


   WOImGui* gui = WOImGui::New( nullptr );
   gui->setLabel( "My Gui" );
   gui->subscribe_drawImGuiWidget(
      [this, gui, light]() //this is a lambda, the capture clause is in [], the input argument list is in (), and the body is in {}
      {
         //ImGui::ShowDemoWindow(); //Displays the default ImGui demo from C:/repos/aburn/engine/src/imgui_implot/implot_demo.cpp
         //WOImGui::draw_AftrImGui_Demo( gui ); //Displays a small Aftr Demo from C:/repos/aburn/engine/src/aftr/WOImGui.cpp
         //ImPlot::ShowDemoWindow(); //Displays the ImPlot demo using ImGui from C:/repos/aburn/engine/src/imgui_implot/implot_demo.cpp

          /*ImGUI Settings Window - Tristan Wellman*/
          {

              static bool shadows = false;
              static float fov = this->cam->getCameraHorizontalFOVDeg();
              static float lightCol[4] = { 1.0f, 1.0f, 1.0f, 1.0f};

              ImGui::SetNextWindowSize(ImVec2(400, 200));
              if (ImGui::Begin("Settings", NULL)) {
 
                  ImGui::Checkbox("Enable Shadows", &shadows);

                  ImGui::BulletText("Main Camera");
                  ImGui::SliderFloat("Camera FOV", &fov, 0.1f, 120.0f);
                  
                  ImGui::BulletText("Global Light");
                  ImGui::ColorEdit4("Light color", lightCol);

              }
              ImGui::End();

              this->cam->setCameraHorizontalFOV(fov);
              this->cam->setCameraVerticalFOV(fov);


              light->setColor(aftrColor4f(lightCol[0], lightCol[1], lightCol[2], lightCol[3]));

              if (shadows && !this->glRenderer->isUsingShadowMapping()) {
                  this->glRenderer->isUsingShadowMapping(true); //set to TRUE to enable shadow mapping, must be using GL 3.2+
              }
              else if (!shadows && this->glRenderer->isUsingShadowMapping()) {
                  this->glRenderer->isUsingShadowMapping(false);
              }

              openFoamRenderer.renderImGuivtkSettings();
          }

      


      } );

   this->worldLst->push_back( gui );




   // remder spitball
   //if(spitball>0) {
       WO* wo = WO::New(shinyRedPlasticCube, Vector(1, 1, 1), MESH_SHADING_TYPE::mstFLAT);
       Vector pos = this->cam->getPosition();
       wo->setPosition(Vector(pos.x+10,pos.y-4,pos.z-4));
       wo->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
       std::string id = "ball";
       wo->setLabel(id);
       worldLst->push_back(wo);
   //}
       this->setActor(wo);

   createNewModuleWayPoints();
}


void GLViewNewModule::createNewModuleWayPoints()
{
   // Create a waypoint with a radius of 3, a frequency of 5 seconds, activated by GLView's camera, and is visible.
   WayPointParametersBase params(this);
   params.frequency = 5000;
   params.useCamera = true;
   params.visible = true;
   WOWayPointSpherical* wayPt = WOWayPointSpherical::New( params, 3 );
   wayPt->setPosition( Vector( 50, 0, 3 ) );
   worldLst->push_back( wayPt );
}
