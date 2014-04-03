#include "ControllerEvent.h"
#include "Controller.h"
#include "Logger.h"
#include <iostream>
#include "math.h" 
#include <fstream>
#include <iomanip>
#include "ControlPosition.cpp" // This controls the position of tool and target using PID.
// #include "ControllerRotation.cpp"


using namespace std;

class MyController : public Controller {  
public:  
  void onInit(InitEvent &evt);  
  double onAction(ActionEvent&);
  void onRecvMsg(RecvMsgEvent &evt); 
  void onCollision(CollisionEvent &evt);   
 
private:
  bool Colli; 
 
}; 

// the first manipulation trial is performed using the preset velocity. then it is updated for each subseqent trial.
// Setting it is important to apply updates to in subseqent trials. 

Vector3d firstVelocityOnTool;
Vector3d firstForceOnTool;

bool multipleSamples = true ; // yes if more than one sample is taken.
int numberOfSamples = 15 ;     // set the number of samples for manipulations. the rules for each sample are set below!
int manipulationsPerSample = 120; // if multiple samples are taken sequentially, the set the number of manipulations per sample.

bool applyVelocity = false;
bool applyForce = true; 


// please set this only if multipleSamples is false!

bool singleSimulation = 0; // if changed to 0, the simulation keeps running. if changed to 1, only one simulation of first sample will run 
int  no_of_simulation = 5; // this works only if singleSimulation is false!

int totalSimulations =   (multipleSamples)  ? ( numberOfSamples * manipulationsPerSample ) : no_of_simulation  ; // that's the total number of times simulation shall run. 


static bool isVelocityinXAllowed ;  // for "Contract Arm " action, only Z component of velocity is set. But change it if you want to apply "Push Diagonally" or "Slide Left"
static bool isVelocityinZAllowed ; // for "Contract Arm " action, only Z component of velocity is set

static int xVelocityVariance; // change the variance in x conmpont of desired velocity. Only works if "isVelocityinXAllowed" flag is set.
static int zVelocityVariance;  // change the variance in Z conmpont of desired velocity. Only works if "isVelocityinZAllowed" flag is set.

static bool doesTargetPositionReset; // reset the target position to the position set in XML world file 
static bool doesToolPositionReset;  // reset the tool position to the position set in XML world file   

static bool doesTargetRotationReset; // reset the target position to the position set in XML world file 
static bool doesToolRotationReset;  // reset the tool position to the position set in XML world file   



// static bool isTargetRotationReset;  // reset the target rotation to the rotation set in XML world file // CURRENTLY: functionality not implemented!
// static bool isToolRotationReset;    // reset the target rotation to the rotation set in XML world file // CURRENTLY: functionality not implemented!

static bool isForceinXAllowed ;  // for "Contract Arm " action, only Z component of force is set. But change it if you want to apply "Push Diagonally" or "Slide Left"
static bool isForceinZAllowed ; // for "Contract Arm " action, only Z component of force is set

static double xForceVariance; // change the variance in x conmpont of desired velocity. Only works if "isVelocityinXAllowed" flag is set.
static double zForceVariance;  // change the variance in Z conmpont of desired velocity. Only works if "isVelocityinZAllowed" flag is set.



static Vector3d velocityOnTool;  // the first manipulation trial is performed using the preset velocity. then it is updated for each subseqent trial.
static Vector3d forceOnTool; // the first manipulation trial is performed using the preset force. then it is updated for each subseqent trial.

static int messageCount;
static int onActionCount;

int actionNumber;
int functionalFeature;

int flag;

static Vector3d pos;
Vector3d initToolPos; // this stores initial position of the tool, used to reset the position when it comes to rest collision with target.
Vector3d toolPos;     // this stores current position of the target, used to reset the position when it comes to rest after being hit by the tool.


Vector3d velocityOfTool;    // gets the velocity of tool.
static int simulationCount=0; // counts how many times, the manipulation trial is performed.
int currentCount=0;
// double pivotSimulation = 843;
// double massOfTool; 

static Vector3d initTargetPos; // this stores initial position of the tool, used to reset the position after collision with target
Vector3d targetPos ; // this stores the changing position of target object.
Vector3d velocityOfTarget; // this stores the velocity of target object

double massOfTool=0;
double massOfObj=0;

static double totalTime=0;

static Vector3d distanceCovered(0,0,0);

ofstream myfile ("example.csv", ios_base::trunc); // empty the already existing content of the file and write it as new!


double refVal_x = 0, refVal_z = 0;  // these variables store the actually applied velocity of tool after P-control is applied.

double netVelocityTool=0;  // net velocity of tool 
double netVelocityTarget=0; // net velocity of target object. 

using namespace std;



Vector3d vel;
Rotation rotTool;

Vector3d toolAngularVel;

double P_Value_X=0;  // P values for x and z component
double P_Value_Z=0;

double refVal_X, refVal_Z; // updated applied velocity in x and x direction


double K_p=1;         // the Kp value is fixed
static double setPoint_X;    // set points, these are the desired velocities
static double setPoint_Y;
static double setPoint_Z;

double v_x = 0; // variable used when tool velocity is randomized
double v_z = 0; // variable used when tool velocity is randomized

Vector3d startingTargetPos;
Vector3d startingToolPos;

bool first = false;
bool isToolPositionReset;
bool isTargetPositionReset;
bool isTargetAtSafePos;

bool isTargetRotationReset;  // reset the target rotation to the rotation set in XML world file // CURRENTLY: functionality not implemented!
bool isToolRotationReset;    // reset the target rotation to the rotation set in XML world file // CURRENTLY: functionality not implemented!



Vector3d differenceInPos;
Vector3d currentVelocity; // current velocity of Tool

Rotation initialToolRot; // initial rotation of the tool
Rotation initialTargetRot; // initial rotation of the target

static double previousTime=0;
double currentTime;
double timeDiff=0;

static int targetCount = 1;

          


void MyController::onInit(InitEvent &evt) {

  first=true;



  SimObj *tool = getObj("StickTool");
  SimObj *box = getObj("box_001");
  tool->setMass(5.0); // the same mass is to be used for all experiments to maintain consistency!

  
  // when multiple samples is not taken and  only single simulation is used 
  if (singleSimulation && !multipleSamples)
  {
     firstVelocityOnTool.set(0,0,20);  // for "Contract Arm" action
     firstForceOnTool.set(0,0,10000);
     
  }

   
  // when multiple samples are not taken, but many simulations of the same sample is taken.  
  if( !singleSimulation && !multipleSamples )

  {

     firstVelocityOnTool.set(-20,0,20);  // for "Pull Diagonally" action
     firstForceOnTool.set(-10000,0,10000);

     isVelocityinXAllowed = 1; // for "Contract Arm " action, only Z component of velocity is set. But change it if you want to apply "Push Diagonally" or "Slide Left"
     isVelocityinZAllowed = 1; // for "Contract Arm " action, only Z component of velocity is set

     xVelocityVariance = 20 ; // change the variance in x conmpont of desired velocity. Only works if "isVelocityinXAllowed" flag is set.
     zVelocityVariance = 20;  // change the variance in Z conmpont of desired velocity. Only works if "isVelocityinZAllowed" flag is set.


     isForceinXAllowed = 1;
     isForceinZAllowed = 1;

     xForceVariance = 2000;
     zForceVariance = 2000;



     doesTargetPositionReset = 1; // reset the target position to the position set in XML world file 
     doesToolPositionReset = 1;   // reset the tool position to the position set in XML world file   

     doesTargetRotationReset = 1; // reset the target position to the position set in XML world file 
     doesToolRotationReset = 1;   // reset the tool position to the position set in XML world file   



  }

  // the value of initTargetPos variable is acquired from XML file in case no multiple samples are used for target manipulation, else they are 
  // used from rules for each sample.
  if (!multipleSamples )
  {
    box->getPosition(initTargetPos);
  }



  velocityOnTool = firstVelocityOnTool;
  forceOnTool    = firstForceOnTool; 

  /* When force is being applied, comment the line below */
  // tool->setLinearVelocity(velocityOnTool.x(), velocityOnTool.y(), velocityOnTool.z() );

  tool->addForce(forceOnTool.x(), forceOnTool.y(), forceOnTool.z());

  setPoint_X = velocityOnTool.x();  // this is the desired velocity in X
  setPoint_Z=  velocityOnTool.z();  // this is the desired velocity in Z

  refVal_X = setPoint_X;    // this is the actually applied feedback velocity in X
  refVal_Z = setPoint_Z;    // this is the actually applied feedback velocity in Z

 
  Colli = false; 
  flag=1;
  isToolPositionReset=true;
  isTargetPositionReset=true;
  isTargetAtSafePos = true;

  isToolRotationReset=true;
  isTargetRotationReset=true;

  differenceInPos.set(0,0,0);
  tool->getPosition(initToolPos);
  tool->getRotation(initialToolRot); // gets initial rotation of tool. 

  box->getRotation(initialTargetRot); 


  // tool->getRotation(rotTool); 
  // box->getRotation(rotObj);

  // This parameter shall be stored for learning affordances using Bayesian Networks !
  LOG_MSG((" Initial Target position onInit() ))is : %f %f %f ", initTargetPos.x(), initTargetPos.y(), initTargetPos.z() ));

  myfile.flush();


  if (myfile.is_open())
  {
     myfile << "InitialToolPosX" << " , " << "InitialToolPosZ" << " , " << "InitialTargetPosX" << " , " << "InitialTargetPosZ" << " , " << "Action" << " , " << "Functional Feature" << "  , ";
     
     if (applyVelocity && !applyForce)
     {
        myfile <<  "xSetVelcoity_tool" << " , " << "zSetVelocity_tool" << " , "; 
     }

     if(applyForce && !applyVelocity)
     {
        myfile <<  "xSetForce_tool" << " , " << "zSetForce_tool" << " , "; 
     }
     
     myfile << "xVelAtContact_Tool" << " , " << "zVelAtContact_Tool" << " , " << "xVelAtContact_Target" << " , " << "zVelAtContact_Target" <<  " , ";
     myfile <<  "targetFinalPosX" << " , " << "targetFinalPosZ"  << " , " << "targetDisplacement_X" << " , " << "targetDisplacement_Z"   <<"\n" ; 

     myfile << setprecision(2) << std::fixed;

     myfile << initToolPos.x() << " , "  << initToolPos.
     z() << " , ";
     myfile << initTargetPos.x() << " , " << initTargetPos.z() << " , " ;

  }

  if (applyForce && applyVelocity)
  {
    cout << "Both force and velocity are being appled. This program shall abort" << endl; 
    exit(0);
  }


 
}  

// --------------------------------

int sampleCount=0;

bool flag_set_parameters = true;
  
double MyController::onAction(ActionEvent &evt) {  
  //return 1.0;
  SimObj *tool = getObj("StickTool");
  SimObj *box = getObj("box_001");


  // cout << "The simulation count in beginning is " << simulationCount << endl;

  double  K_p_tool= 9;
  double  K_i_tool= 1.0;
  double  K_d_tool= 4.2;
  double  gain_tool = 140000;




  if( multipleSamples && !singleSimulation )
  {

    if (simulationCount == 0 || ((simulationCount) % manipulationsPerSample) == 0 )
      {
        flag_set_parameters = true; 
      }
    else {
        flag_set_parameters = false;
      }

  }

  if ( multipleSamples && !singleSimulation )
  {

      /* The position and orientation reset for tool and target object is common to all samples */ 

      doesTargetPositionReset = 1;   // reset the target position to the position required for each sample.
      doesToolPositionReset   = 1 ;  // reset the tool position to the position required for each sample. 

      doesTargetRotationReset = 1; // reset the target position to the position set in XML world file 
      doesToolRotationReset   = 1;   // reset the tool position to the position set in XML world file   


   
    // Rule 1: for contract Arm using horizontal part  

    if  ( (0 <= simulationCount ) && (simulationCount < manipulationsPerSample )  )
    {

          // cout << " first loop simulation count " << simulationCount <<  endl; 

          actionNumber = 1; 
          functionalFeature = 1; 

          if ( simulationCount == 0)
          {
               firstVelocityOnTool.set(0,0,20);
               velocityOnTool = firstVelocityOnTool;

              firstForceOnTool.set(0,0,3000);
              forceOnTool = firstForceOnTool; 

          }


          isVelocityinXAllowed = 0; 
          isVelocityinZAllowed = 1;

          // xVelocityVariance = 20 ; // change the variance in x conmpont of desired velocity. Only works if "isVelocityinXAllowed" flag is set.
          zVelocityVariance = 20;  // change the variance in Z conmpont of desired velocity. Only works if "isVelocityinZAllowed" flag is set.


          isForceinXAllowed = 0;
          isForceinZAllowed = 1;

          xForceVariance = 2000;
          zForceVariance = 2000;


          initTargetPos.set(-50,1,50.0);
         
    }

     // Rule 2: for contract Arm using vertical part   

   else  if  ( ( (manipulationsPerSample * 1) <= simulationCount ) && (simulationCount < (manipulationsPerSample * 2 ) )   )  
    {

          // cout << " first loop simulation count " << simulationCount <<  endl; 

        actionNumber = 1; 
        functionalFeature = 2; 


        if (simulationCount == (manipulationsPerSample *1 ))
        {


          firstVelocityOnTool.set(0,0,20);
          velocityOnTool = firstVelocityOnTool;

          firstForceOnTool.set(0,0,3000);
          forceOnTool = firstForceOnTool; 
        }


          isVelocityinXAllowed = 0; 
          isVelocityinZAllowed = 1;

          // xVelocityVariance = 20 ; // change the variance in x conmpont of desired velocity. Only works if "isVelocityinXAllowed" flag is set.
          zVelocityVariance = 20;  // change the variance in Z conmpont of desired velocity. Only works if "isVelocityinZAllowed" flag is set.

          isForceinXAllowed = 0;
          isForceinZAllowed = 1;

          xForceVariance = 2000;
          zForceVariance = 2000;


          initTargetPos.set(-10.0,1,55); // Only this will change here!
         
    }

     // Rule 3: for contract Arm using corner    

    else  if  ( ( (manipulationsPerSample * 2) <= simulationCount ) && (simulationCount < (manipulationsPerSample * 3 ) )   )  
    {

          // cout << " first loop simulation count " << simulationCount <<  endl; 

           actionNumber = 1; 
           functionalFeature = 3; 

         if (simulationCount == (manipulationsPerSample * 2 ))
        {

          firstVelocityOnTool.set(0,0,20);
          velocityOnTool = firstVelocityOnTool;

          firstForceOnTool.set(0,0,3000);
          forceOnTool = firstForceOnTool; 

        }


          isVelocityinXAllowed = 0; 
          isVelocityinZAllowed = 1;

          // xVelocityVariance = 20 ; // change the variance in x conmpont of desired velocity. Only works if "isVelocityinXAllowed" flag is set.
          zVelocityVariance = 20;  // change the variance in Z conmpont of desired velocity. Only works if "isVelocityinZAllowed" flag is set.

          isForceinXAllowed = 0;
          isForceinZAllowed = 1;

          xForceVariance = 2000;
          zForceVariance = 2000;

          initTargetPos.set( -10.0, 1, 50.0 ); // Only this will change here for the corner
          // initTargetPos.set( 0.0, 1, 55.0 ); // Only this will change here for the corner, when the tool is on 45degree to original..s

          // static Rotation initialToolRot( 0.9239557,  0.0,  0.3824995,  0.0 );

          // initialToolRot.qw() = 0.9239557;
          // initialToolRot.qx() = 0.0;
          // initialToolRot.qy() = 0.3824995;
          // initialToolRot.qz() = 0.0; 

          // initialToolRot.qw(0.9239557);
          // initialToolRot.qx(0.0);
          // initialToolRot.qy(0.3824995);
          // initialToolRot.qz() = 0.0; 
         
    }  

    // Rule 4: for slide left arm with horizotal part feature 
  
    else  if  ( ( (manipulationsPerSample * 3) <= simulationCount ) && (simulationCount < (manipulationsPerSample * 4 ) )   )  

     {

         actionNumber = 2; 
         functionalFeature = 1; 

      if (simulationCount == (manipulationsPerSample * 3 ))
        {
    
          firstVelocityOnTool.set(-20,0,0);
          velocityOnTool = firstVelocityOnTool;

          firstForceOnTool.set(-3000,0,0);
          forceOnTool = firstForceOnTool; 

        }

          isVelocityinXAllowed = 1; 
          isVelocityinXAllowed = 0;

          xVelocityVariance = -20 ; // change the variance in x conmpont of desired velocity. Only works if "isVelocityinXAllowed" flag is set.
          // zVelocityVariance = 20;  // change the variance in Z conmpont of desired velocity. Only works if "isVelocityinZAllowed" flag is set.

          isForceinXAllowed = 1;
          isForceinZAllowed = 0;

          xForceVariance = -2000;
          zForceVariance = 2000;


          initTargetPos.set(-15,1,50.0); 
    
          
    }

    // Rule 5: for slide left arm with vertical part feature 
  
    else  if  ( ( (manipulationsPerSample * 4) <= simulationCount ) && (simulationCount < (manipulationsPerSample * 5 ) )   )  

     {

       actionNumber = 2; 
       functionalFeature = 2; 


      if (simulationCount == (manipulationsPerSample * 4 ))
        {
    
          firstVelocityOnTool.set(-20,0,0);
          velocityOnTool = firstVelocityOnTool;

          firstForceOnTool.set(-3000,0,0);
          forceOnTool = firstForceOnTool; 
       }

          isVelocityinXAllowed = 1; 
          isVelocityinXAllowed = 0;

          xVelocityVariance = -20 ; // change the variance in x conmpont of desired velocity. Only works if "isVelocityinXAllowed" flag is set.
          // zVelocityVariance = 20;  // change the variance in Z conmpont of desired velocity. Only works if "isVelocityinZAllowed" flag is set.


          isForceinXAllowed = 1;
          isForceinZAllowed = 0;

          xForceVariance = -2000;
          zForceVariance = 2000;


          initTargetPos.set(-10.0,1,90); // Only this will change here for the vertical part
    
          
    }

    // Rule 6: for slide left arm with corner feature 
  
    else  if  ( ( (manipulationsPerSample * 5) <= simulationCount ) && (simulationCount < (manipulationsPerSample * 6 ) )   )  

     {

       actionNumber = 2; 
       functionalFeature = 3; 

    
      if (simulationCount == (manipulationsPerSample * 5 ))
        {
          firstVelocityOnTool.set(-20,0,0);
          velocityOnTool = firstVelocityOnTool;


          firstForceOnTool.set(-3000,0,0);
          forceOnTool = firstForceOnTool; 
        }

          isVelocityinXAllowed = 1; 
          isVelocityinXAllowed = 0;

          xVelocityVariance = -20 ; // change the variance in x conmpont of desired velocity. Only works if "isVelocityinXAllowed" flag is set.
          // zVelocityVariance = 20;  // change the variance in Z conmpont of desired velocity. Only works if "isVelocityinZAllowed" flag is set.

          isForceinXAllowed = 1;
          isForceinZAllowed = 0;

          xForceVariance = -2000;
          zForceVariance = 2000;


          initTargetPos.set(-10.0,1,50.0); // Only this will change here for the corner

          // initTargetPos.set(4.0,1,40.0); // Only this will change here for the corner
           
    }


    // Rule 7: for pull arm diagonally with horizontal part feature! 
    else if ( ( (manipulationsPerSample * 6)  <= simulationCount ) && (simulationCount < (manipulationsPerSample * 7))  ) {

         actionNumber = 3; 
         functionalFeature = 1; 


         if (simulationCount == (manipulationsPerSample * 6 ))
        {


          firstVelocityOnTool.set(-20,0,20);
          velocityOnTool = firstVelocityOnTool;

          firstForceOnTool.set(-3000,0, 3000);
          forceOnTool = firstForceOnTool; 

        }
          isVelocityinXAllowed = 1; 
          isVelocityinXAllowed = 1;
  
          xVelocityVariance = -20 ; // change the variance in x conmpont of desired velocity. Only works if "isVelocityinXAllowed" flag is set.
          zVelocityVariance = 20;  // change the variance in Z conmpont of desired velocity. Only works if "isVelocityinZAllowed" flag is set.

          isForceinXAllowed = 1;
          isForceinZAllowed = 1;

          xForceVariance = -2000;
          zForceVariance = 2000;

          initTargetPos.set(-70,1,50.0);

          K_p_tool= 9;
          K_i_tool= 1.4;
          K_d_tool= 4;
          gain_tool = 150000;
  
  
          
    }

    // Rule 8: for pull arm diagonally with vertical part feature! 
    else if ( ( (manipulationsPerSample * 7)  <= simulationCount ) && (simulationCount < (manipulationsPerSample * 8))  ) {

           actionNumber = 3; 
           functionalFeature = 2; 

        if (simulationCount == (manipulationsPerSample * 7 ))
        {


          firstVelocityOnTool.set(-20,0,20);
          velocityOnTool = firstVelocityOnTool;


          firstForceOnTool.set(-3000,0, 3000);
          forceOnTool = firstForceOnTool; 

        }

          isVelocityinXAllowed = 1; 
          isVelocityinXAllowed = 1;
  
          xVelocityVariance = -20 ; // change the variance in x conmpont of desired velocity. Only works if "isVelocityinXAllowed" flag is set.
          zVelocityVariance = 20;  // change the variance in Z conmpont of desired velocity. Only works if "isVelocityinZAllowed" flag is set.

          isForceinXAllowed = 1;
          isForceinZAllowed = 1;

          xForceVariance = -2000;
          zForceVariance = 2000;

          initTargetPos.set(-10.0,1,110);

          K_p_tool= 9;
          K_i_tool= 1.4;
          K_d_tool= 4;
          gain_tool = 150000;
          
    }


    // Rule 9: for pull arm diagonally with corner feature! 

    else if ( ( (manipulationsPerSample * 8)  <= simulationCount ) && (simulationCount < (manipulationsPerSample * 9))  ) {

           actionNumber = 3; 
           functionalFeature = 3; 

         if (simulationCount == (manipulationsPerSample * 8 ))
        {
          firstVelocityOnTool.set(-20,0,20);
          velocityOnTool = firstVelocityOnTool;

          firstForceOnTool.set(-3000,0, 3000);
          forceOnTool = firstForceOnTool; 

        }

          isVelocityinXAllowed = 1; 
          isVelocityinXAllowed = 1;
  
          xVelocityVariance = -20 ; // change the variance in x conmpont of desired velocity. Only works if "isVelocityinXAllowed" flag is set.
          zVelocityVariance = 20;  // change the variance in Z conmpont of desired velocity. Only works if "isVelocityinZAllowed" flag is set.

          isForceinXAllowed = 1;
          isForceinZAllowed = 1;

          xForceVariance = -2000;
          zForceVariance = 2000;

          initTargetPos.set(-10.0,1,50.0);

          K_p_tool= 9;
          K_i_tool= 1.4;
          K_d_tool= 4;
          gain_tool = 150000;
  
          
    }

    // Rule 10: for Slide Arm Right with Horizontal part! 

     else if ( ( (manipulationsPerSample * 9)  <= simulationCount ) && (simulationCount < (manipulationsPerSample * 10))  ) {

           actionNumber = 4; 
           functionalFeature = 1; 

         if (simulationCount == (manipulationsPerSample * 9 ))
        {
          firstVelocityOnTool.set(20,0,0);
          velocityOnTool = firstVelocityOnTool;

          firstForceOnTool.set(3000,0, 0);
          forceOnTool = firstForceOnTool; 

        }

          isVelocityinXAllowed = 1; 
          isVelocityinXAllowed = 0;
  
          xVelocityVariance = 20 ; // change the variance in x conmpont of desired velocity. Only works if "isVelocityinXAllowed" flag is set.
          zVelocityVariance = 20;  // change the variance in Z conmpont of desired velocity. Only works if "isVelocityinZAllowed" flag is set.

          isForceinXAllowed = 1;
          isForceinZAllowed = 0;

          xForceVariance = 2000;
          zForceVariance = 2000;

          initTargetPos.set(15.0,1,50.0);

          K_p_tool= 9;
          K_i_tool= 1.4;
          K_d_tool= 4;
          gain_tool = 150000;
  
          
    }



    // Rule 11: for Slide Arm Right with Vertical! 

     else if ( ( (manipulationsPerSample * 10)  <= simulationCount ) && (simulationCount < (manipulationsPerSample * 11))  ) {

           actionNumber = 4; 
           functionalFeature = 2; 

         if (simulationCount == (manipulationsPerSample * 10 ))
        {
          firstVelocityOnTool.set(20,0,0);
          velocityOnTool = firstVelocityOnTool;

          firstForceOnTool.set(3000,0, 0);
          forceOnTool = firstForceOnTool; 

        }

          isVelocityinXAllowed = 1; 
          isVelocityinXAllowed = 0;
  
          xVelocityVariance = 20 ; // change the variance in x conmpont of desired velocity. Only works if "isVelocityinXAllowed" flag is set.
          zVelocityVariance = 20;  // change the variance in Z conmpont of desired velocity. Only works if "isVelocityinZAllowed" flag is set.

          isForceinXAllowed = 1;
          isForceinZAllowed = 0;

          xForceVariance = 2000;
          zForceVariance = 2000;

          initTargetPos.set(10.0,1,90.0);

          K_p_tool= 9;
          K_i_tool= 1.4;
          K_d_tool= 4;
          gain_tool = 150000;
  
          
    }




    // Rule 12: for Slide Arm Right with Corner! 

     else if ( ( (manipulationsPerSample * 11)  <= simulationCount ) && (simulationCount < (manipulationsPerSample * 12))  ) {

           actionNumber = 4; 
           functionalFeature = 3; 

         if (simulationCount == (manipulationsPerSample * 11 ))
        {
          firstVelocityOnTool.set(20,0,0);
          velocityOnTool = firstVelocityOnTool;

          firstForceOnTool.set(3000,0, 0);
          forceOnTool = firstForceOnTool; 

        }

          isVelocityinXAllowed = 1; 
          isVelocityinXAllowed = 0;
  
          xVelocityVariance = 20 ; // change the variance in x conmpont of desired velocity. Only works if "isVelocityinXAllowed" flag is set.
          zVelocityVariance = 20;  // change the variance in Z conmpont of desired velocity. Only works if "isVelocityinZAllowed" flag is set.

          isForceinXAllowed = 1;
          isForceinZAllowed = 0;

          xForceVariance = 2000;
          zForceVariance = 2000;

          initTargetPos.set(10.0,1,50.0);

          K_p_tool= 9;
          K_i_tool= 1.4;
          K_d_tool= 4;
          gain_tool = 150000;
  
          
    }




    // Rule 13: for Pull-Diagoally-2 with Horizontal part! 

    else if ( ( (manipulationsPerSample * 12)  <= simulationCount ) && (simulationCount < (manipulationsPerSample * 13))  ) {

           actionNumber = 5; 
           functionalFeature = 1; 

         if (simulationCount == (manipulationsPerSample * 12 ))
        {
          firstVelocityOnTool.set(20,0,20);
          velocityOnTool = firstVelocityOnTool;

          firstForceOnTool.set(3000,0, 3000);
          forceOnTool = firstForceOnTool; 

        }

          isVelocityinXAllowed = 1; 
          isVelocityinXAllowed = 0;
  
          xVelocityVariance = 20 ; // change the variance in x conmpont of desired velocity. Only works if "isVelocityinXAllowed" flag is set.
          zVelocityVariance = 20;  // change the variance in Z conmpont of desired velocity. Only works if "isVelocityinZAllowed" flag is set.

          isForceinXAllowed = 1;
          isForceinZAllowed = 1;

          xForceVariance = 2000;
          zForceVariance = 2000;

          initTargetPos.set(70.0,1,50.0);

          K_p_tool= 9;
          K_i_tool= 1.4;
          K_d_tool= 4;
          gain_tool = 150000;
  
          
    }




    // Rule 14: for Pull-Diagonally-2 with Vertical! 

    else if ( ( (manipulationsPerSample * 13)  <= simulationCount ) && (simulationCount < (manipulationsPerSample * 14))  ) {

           actionNumber = 5; 
           functionalFeature = 2; 

         if (simulationCount == (manipulationsPerSample * 13 ))
        {
          firstVelocityOnTool.set(20,0,20);
          velocityOnTool = firstVelocityOnTool;

          firstForceOnTool.set(3000,0, 3000);
          forceOnTool = firstForceOnTool; 

        }

          isVelocityinXAllowed = 1; 
          isVelocityinXAllowed = 1;
  
          xVelocityVariance = 20 ; // change the variance in x conmpont of desired velocity. Only works if "isVelocityinXAllowed" flag is set.
          zVelocityVariance = 20;  // change the variance in Z conmpont of desired velocity. Only works if "isVelocityinZAllowed" flag is set.

          isForceinXAllowed = 1;
          isForceinZAllowed = 1;

          xForceVariance = 2000;
          zForceVariance = 2000;

          initTargetPos.set(10.0, 1, 110.0);

          K_p_tool= 9;
          K_i_tool= 1.4;
          K_d_tool= 4;
          gain_tool = 150000;
  
          
    }



    // Rule 15: for Pull-Diagoanlly-2 with Corner! 

    else if ( ( (manipulationsPerSample * 14)  <= simulationCount ) && (simulationCount < (manipulationsPerSample * 15))  ) {

        actionNumber = 5; 
        functionalFeature = 3; 

         if (simulationCount == (manipulationsPerSample * 14 ))
        {
          firstVelocityOnTool.set(20,0,20);
          velocityOnTool = firstVelocityOnTool;

          firstForceOnTool.set(3000,0, 3000);
          forceOnTool = firstForceOnTool; 

        }

          isVelocityinXAllowed = 1; 
          isVelocityinXAllowed = 1;
  
          xVelocityVariance = 20 ; // change the variance in x conmpont of desired velocity. Only works if "isVelocityinXAllowed" flag is set.
          zVelocityVariance = 20;  // change the variance in Z conmpont of desired velocity. Only works if "isVelocityinZAllowed" flag is set.

          isForceinXAllowed = 1;
          isForceinZAllowed = 1;

          xForceVariance = 2000;
          zForceVariance = 2000;

          initTargetPos.set(10.0,1,50.0);

          K_p_tool= 9;
          K_i_tool= 1.4;
          K_d_tool= 4;
          gain_tool = 150000;
  
          
    }












    else {
      cout << "The limit of manipulation samples is reached " << endl;
      // exit(0);
    }

  }

  totalTime=evt.time();
  tool->getLinearVelocity(vel);
  // LOG_MSG((" Current tool velocity is  : %f %f %f ", vel.x(), vel.y(), vel.z() ));

  // add velocity till it hits the target object
  // This parameter shall be stored for learning affordances using Bayesian Networks !


  if (Colli == false &&  ( (doesToolPositionReset && isToolPositionReset) || !doesToolPositionReset ) &&  ( (doesTargetPositionReset && isTargetPositionReset) || !doesTargetPositionReset ) ) {

     if (flag==1){


        tool->getPosition(startingToolPos);
        LOG_MSG((" Starting position of tool is : %f %f %f ", startingToolPos.x(), startingToolPos.y(), startingToolPos.z() ));

        if (myfile.is_open())

          {
            myfile << actionNumber << " , " << functionalFeature << " , " ;
          }

        if( !doesToolPositionReset && !singleSimulation && !first  )

        {
           
           if (myfile.is_open())

          {

            
            myfile << startingToolPos.x() << " , " << startingToolPos.z() << " , " ;
          }

        }


        
        box->getPosition(startingTargetPos);
        LOG_MSG((" Starting position of target is : %f %f %f ", startingTargetPos.x(), startingTargetPos.y(), startingTargetPos.z() ));

     

        if (!doesTargetPositionReset && !singleSimulation && !first )

        {

         if (myfile.is_open())
          {
            myfile << startingTargetPos.x() << " , " << startingTargetPos.z() << " , " ;
          }

        }



        LOG_MSG(("Setpoint Velocity on tool object is : %f %f %f ", velocityOnTool.x(), velocityOnTool.y(), velocityOnTool.z() ));
        if (myfile.is_open() && applyVelocity && !applyForce)
        {
          myfile << velocityOnTool.x() << " , " <<velocityOnTool.z() << " , "; 
        }

        LOG_MSG(("Setpoint force on tool is : %f %f %f ", forceOnTool.x(), forceOnTool.y(), forceOnTool.z() ));
        if (myfile.is_open() && applyForce && !applyVelocity )
        {
          myfile << forceOnTool.x() << " , " << forceOnTool.z() << " , "; 
        }




        flag=0;
     
      }

     // The P-Controller is applied to control the velocity 

     P_Value_X = K_p * (setPoint_X - vel.x() ); 
     P_Value_Z = K_p * (setPoint_Z - vel.z() ); 

     refVal_X  = refVal_X + P_Value_X; 
     refVal_Z  = refVal_Z + P_Value_Z;

     // cout << "Added velocity in X is = " << refVal_X + P_Value_X << endl;
     // cout << "Added velocity in Z is = " << refVal_Z + P_Value_Z << endl;

     //tool->setLinearVelocity(refVal_X, 0, refVal_Z);  // this is the velocity applied after taking feedback from P-controller.
      
     // tool->setLinearVelocity(velocityOnTool.x(), velocityOnTool.y(), velocityOnTool.z() ); // without applying P-Controller. 

     tool->addForce(forceOnTool.x(), forceOnTool.y(), forceOnTool.z()  );
     // tool->addTorque(0,10000,0); // for testing the torque controller.
 
   }



  // get the net velocity of tool 

  tool->getLinearVelocity(velocityOfTool);
  
  netVelocityTool=( pow(velocityOfTool.x(),2) + pow(velocityOfTool.y(), 2) + pow(velocityOfTool.z(), 2 ) );
  netVelocityTool = sqrt(netVelocityTool);




  // get the net velocity of target object

  box->getLinearVelocity(velocityOfTarget);
  netVelocityTarget=( pow(velocityOfTarget.x(),2) + pow(velocityOfTarget.y(), 2) + pow(velocityOfTarget.z(), 2 ) );
  netVelocityTarget = sqrt(netVelocityTarget);


  // The folliwung if loop checks if tool and target have come to test. Then the final target displacement is 
  // calculated and position of tool and target are reset
  // to their respective initial positions
 
  if (Colli == true)
  {
  
     if (netVelocityTarget < 0.001 && netVelocityTool < 0.001 )
     {
        

        box->getPosition(targetPos);

        // This parameter shall be stored for learning affordances using Bayesian Networks !
        // LOG_MSG((" The final target position at manipulation trial %d is : %f %f %f ", simulationCount, targetPos.x(), targetPos.y(), targetPos.z() ));

        if (myfile.is_open())
          {

            if( ( ( (manipulationsPerSample * 1) < simulationCount ) && (simulationCount <= (manipulationsPerSample * 2 ) )   ) || 
              ( ( (manipulationsPerSample * 3) < simulationCount ) && (simulationCount <= (manipulationsPerSample * 4 ) )   ) ||
              ( ( (manipulationsPerSample * 9) < simulationCount ) && (simulationCount <= (manipulationsPerSample * 10 ) )   )    )
            {
               targetPos.x(startingTargetPos.x());
               targetPos.z(startingTargetPos.z()); 
               // myfile << targetPos.x() << " , " << targetPos.z() << " , " ;
               myfile << 0.0  << " , " << 0.0 << " , " ;
            }

            else{
              box->getPosition(targetPos);
              myfile << targetPos.x() << " , " << targetPos.z() << " , " ;
            }

            // myfile << targetPos.x() << " , " << targetPos.z() << " , " ;
       
          }



        // LOG_MSG((" The displacement from initial position is is : %f %f %f ", targetPos.x()-initTargetPos.x(), targetPos.y()-initTargetPos.y(), targetPos.z()-initTargetPos.z() ));

        // LOG_MSG((" The target displacement from initial position is is : %f %f %f ", targetPos.x()-startingTargetPos.x(), targetPos.y()-startingTargetPos.y(), targetPos.z()-startingTargetPos.z() ));

        // The displacement is calculated from initial target position, so if target reset does not happen accurately, we shall have false value of displacement.

        // if (myfile.is_open())
        //   {
        //     myfile << targetPos.x()-initTargetPos.x() << "," << targetPos.z()-initTargetPos.z() << " \n " ;
        //   }

        if (myfile.is_open())
          {
            myfile << targetPos.x()-startingTargetPos.x() << " , " << targetPos.z()-startingTargetPos.z() << " \n " ;
          }




       while (doesTargetPositionReset && !singleSimulation && !isTargetAtSafePos && ( simulationCount < totalSimulations) && !isToolRotationReset)
        {

    
          // box->getPosition(targetPos);
          // Vector3d safePos(-480,1,480);
          // // // This parameter shall be stored for learning affordances using Bayesian Networks !
          // // // LOG_MSG((" Target pos is : %f %f %f ", targetPos.x(), targetPos.y(), targetPos.z() ));

          // //  // Storing the reset position into the file. This is not required for learning BN, but just for checks. 



          // differenceInPos.x(safePos.x() - targetPos.x());
          // differenceInPos.y(safePos.y() - targetPos.y());
          // differenceInPos.z(safePos.z() - targetPos.z());
          // // LOG_MSG (("The difference in position is: %f, %f, %f ", differenceInPos.x(), differenceInPos.y(), differenceInPos.z() )) ;   

          // // // box->getLinearVelocity(currentVelocity);  
          // // // LOG_MSG (("The target currentVelocity is: %f, %f, %f ", currentVelocity.x(), currentVelocity.y(), currentVelocity.z() )) ; 
          
          // // // double* ptr2 = controlPosition(initTargetPos, targetPos, 1.0,0.0, 0.7);
          // double* ptr2 = controlPosition(safePos, targetPos, 3.0,0.4, 1.2);
          // box->setLinearVelocity( ptr2[0] , 0, ptr2[1]);

          // if ( abs(differenceInPos.x()) < 0.3 && abs(differenceInPos.z()) < 0.3 )
          //   {

          //     // LOG_MSG((" Target reset at : %f %f %f ", targetPos.x(), targetPos.y(), targetPos.z() ));
          //     LOG_MSG (("The difference in target position is: %f, %f, %f ", differenceInPos.x(), differenceInPos.y(), differenceInPos.z() )) ; 
          //     isTargetAtSafePos = true;
          //     differenceInPos.set(0,0,0);
          //     currentVelocity.set(0,0,0);

          //   }

          isTargetAtSafePos=true;

        }   




        while (doesToolRotationReset && !isToolRotationReset && !singleSimulation && isTargetAtSafePos && !isToolPositionReset  && ( simulationCount < totalSimulations) )
        {
           // orientation reset functionality has not been implemented yet!

          Rotation currentToolRot;
          tool->getRotation(currentToolRot);
          // LOG_MSG((" Current Tool orientation is : %f %f %f %f ", currentToolRot.qw(), currentToolRot.qx(), currentToolRot.qy(), currentToolRot.qz() ));
          // LOG_MSG((" Initial Tool orientation is : %f %f %f %f ", initialToolRot.qw(), initialToolRot.qx(), initialToolRot.qy(), initialToolRot.qz() ));
          // isToolRotationReset = true;
          double* ptr = NULL;

          // if ( actionNumber == 1 && functionalFeature == 3 )  
          //    {
          //       Rotation setToolRot( 0.9239557,  0.0,  0.3824995,  0.0 );
          //       ptr = controlRotation(setToolRot, currentToolRot, 2.0,0.4,1.2);

          //     }

          // else if ( actionNumber == 2 && functionalFeature == 3)
          //  {
          //       Rotation setToolRot2( 0.9239557,  0.0,  -0.3824995,  0.0 );
          //       ptr = controlRotation(setToolRot2, currentToolRot, 5.0,0.4,2.2);
          //  }    
             //    cout << "pid_ori_tool = " << ptr[0] << endl; 
             //    tool->addTorque(0, 40000 * ptr[0],0);

             //      if ( abs(ptr[0]) < 0.8 )
             //      {
             //        cout << "The difference in tool orientation is " << ptr[1] << endl;
             //        isToolRotationReset=true;
             //      }

             // }

          // else if ( ( (manipulationsPerSample * 5) <= simulationCount ) && (simulationCount < (manipulationsPerSample * 6 ) )   ) 
          // {
          //   Rotation setToolRot( 0.9239557,  0.0,  -0.3824995,  0.0 );
          //   ptr = controlRotation(setToolRot, currentToolRot, 5.0,0.4,2.2);
          // }    


             
          // else{
                 // ptr = controlRotation(initialToolRot, currentToolRot, 5.0,0.4,2.2);

                cout << "For tool " << endl;

                ptr = controlRotation(initialToolRot, currentToolRot, K_p_tool , K_i_tool, K_d_tool);

              // }
                 // cout << "pid_ori_tool_x = " << ptr[0] << endl; 
                 cout << "pid_ori_tool_y = " << ptr[1] << endl; 
                 // cout << "pid_ori_tool_z = " << ptr[2] << endl; 


                 tool->addTorque(0, gain_tool * ptr[1],0);

                  if ( abs(ptr[1]) < 0.04 )
                  {
                    cout << "The difference in tool orientation about y axis is " << ptr[3] << endl;
                    isToolRotationReset=true;
                  }

          // }

          
         
          
        }

     

      while (doesTargetRotationReset && !isTargetRotationReset && isToolRotationReset && !singleSimulation && ( simulationCount < totalSimulations) )
        {
           // orientation reset functionality has not been implemented yet!

        //   Rotation currentTargetRot;
        //   box->getRotation(currentTargetRot);
        //   cout << " The target rotation = " << currentTargetRot.qw() << " , " << currentTargetRot.qx() << " , " << currentTargetRot.qy() << " ,  " << currentTargetRot.qz() << endl;


        //   double* ptrTarget = NULL;

        //   // cout << "For Target" << endl;
      
        //   ptrTarget = controlTargetRotation(initialTargetRot, currentTargetRot,3.0,0.4,1.2);

        //   cout << "pid_ori_target_x = " << ptrTarget[0] << endl; 
        //   cout << "pid_ori_target_y = " << ptrTarget[1] << endl; 
        //   cout << "pid_ori_target_z = " << ptrTarget[2] << endl; 

        //   int index;

        // if (targetCount == 1) {

        //   index = int (ptrTarget[3]);
        //   cout << "The index at target" << index << endl;

        // }





        //   if ( index == 0 )
        //   {
        //        box->addTorque(200 * ptrTarget[0], 0, 0);
        //        if (  abs(ptrTarget[0])   < 0.03 )
        //         {
        //         // cout << "The difference in target orientation about Y axis is " << ptrTarget[3] << endl;
        //         isTargetRotationReset = true;
        //       }
        //   }

        //   if ( index == 1 )
        //   {
        //       box->addTorque(0, 200 * ptrTarget[1],  0);

        //       if (  abs(ptrTarget[1])   < 0.03 )
        //         {
        //         // cout << "The difference in target orientation about Y axis is " << ptrTarget[3] << endl;
        //         isTargetRotationReset = true;
        //       }

        //   }

        //   if ( index == 2 )
        //   {
        //       box->addTorque(0, 0, 200 * ptrTarget[2] );
        //       if (  abs(ptrTarget[2])   < 0.03 )
        //         {
        //         // cout << "The difference in target orientation about Y axis is " << ptrTarget[3] << endl;
        //         isTargetRotationReset = true;
        //       }

        //       targetCount++;


        // }



          // if (  abs(ptrTarget[1])   < 0.04 )
          // {
          //   cout << "The difference in target orientation about Y axis is " << ptrTarget[3] << endl;
          //   isTargetRotationReset = true;
          // }

          isTargetRotationReset = true;
          
        }


        // // I want the position of tool to reset, once it has come to rest after hitting the target object. 


        // // Please comment out the following four lines for reseting tool to original position 

        while  ( doesToolPositionReset && !singleSimulation && !isToolPositionReset  && ( simulationCount < totalSimulations) && isToolRotationReset && isTargetRotationReset)
        {

            // tool->setPosition(initToolPos.x(), initToolPos.y(), initToolPos.z()); // reset the tool position.

            tool->getPosition(toolPos);
            // This parameter shall be stored for learning affordances using Bayesian Networks !
            // LOG_MSG((" Tool pos is : %f %f %f ", toolPos.x(), toolPos.y(), toolPos.z() ));

            differenceInPos.x(initToolPos.x() - toolPos.x());
            differenceInPos.y(initToolPos.y() - toolPos.y());
            differenceInPos.z(initToolPos.z() - toolPos.z());
            // LOG_MSG (("The difference in position is: %f, %f, %f ", differenceInPos.x(), differenceInPos.y(), differenceInPos.z() )) ;   

            tool->getLinearVelocity(currentVelocity);  
            // LOG_MSG (("The currentVelocity is: %f, %f, %f ", currentVelocity.x(), currentVelocity.y(), currentVelocity.z() )) ; 
          
            
            // double* ptr1 = controlPosition(initToolPos, toolPos, 1.0,0.0, 0.7);
            double* ptr1 = controlPosition(initToolPos, toolPos, 3.0,0.4, 1.2);
            tool->setLinearVelocity( ptr1[0] , 0, ptr1[1]);

            if ( abs(differenceInPos.x()) < 0.05 && abs(differenceInPos.z()) < 0.05)
            {

              // LOG_MSG((" Tool reset at : %f %f %f ", toolPos.x(), toolPos.y(), toolPos.z() ));
              LOG_MSG (("The difference in position is: %f, %f, %f ", differenceInPos.x(), differenceInPos.y(), differenceInPos.z() )) ; 
              isToolPositionReset = true;
              differenceInPos.set(0,0,0);
              currentVelocity.set(0,0,0);

              if (myfile.is_open())
              {
                myfile << toolPos.x() << " , " << toolPos.z() << " , " ;
              }

            }


            
        }

         // // Please comment out the following four lines for reseting target to original position

        int flag1 = 0 ;
        int flag2 = 1 ;

        while (doesTargetPositionReset && !singleSimulation && !isTargetPositionReset && isToolPositionReset && ( simulationCount < totalSimulations) && isTargetRotationReset  )
        {

    
          box->getPosition(targetPos);
          int quadrant;
         
          static Vector3d subgoal1;
          static Vector3d subgoal2;
          static Vector3d targetIntermediatePos;
          // This parameter shall be stored for learning affordances using Bayesian Networks !
          // LOG_MSG((" Target pos is : %f %f %f ", targetPos.x(), targetPos.y(), targetPos.z() ));

           // Storing the reset position into the file. This is not required for learning BN, but just for checks. 

          Vector3d distanceFromOrigin;
          distanceFromOrigin.x( targetPos.x() - initToolPos.x()  );
          distanceFromOrigin.y( targetPos.y() - initToolPos.y()  );
          distanceFromOrigin.z( targetPos.z() - initToolPos.z()  );

          if (distanceFromOrigin.x() > 0 && distanceFromOrigin.z() <  0 )
          {
            quadrant = 1;
            cout << "The quadrant = " << quadrant << endl ;

            // if (!flag1)
            // {
              subgoal1.set(20,1,-160);
            // }
            // if (!flag2){

            //   cout << "It entered the Flag2 = " << flag << endl;
              subgoal2.set(-120 ,1, -160);
            // }
            

          }

          if ( distanceFromOrigin.x() < 0 && distanceFromOrigin.z() <  0 )
          {
            
            quadrant = 2;
            cout << "The quadrant = " << quadrant << endl ;

            // if (!flag1)
            // {
              subgoal1.set(-120,1,-160);
            // }
            // if (!flag2){

              // cout << "It entered the Flag2 = " << flag << endl;
              subgoal2.set(-120, 1, 160);
            // }

          }

         if ( distanceFromOrigin.x() <= 0 && distanceFromOrigin.z() >= 0 )

          {
            quadrant = 3;
            cout << "The quadrant = " << quadrant << endl ;
            cout << "The simulation count at quadrant 3 is " << simulationCount <<  endl;

            if (actionNumber == 4 || actionNumber == 5)

            {
              cout << "It will be moved to quadrant 4" << endl;
              subgoal1.set(-120,1,160);
              subgoal2.set(50,1,160);
            }

            else{

              subgoal1.set(initTargetPos.x(), initTargetPos.y(), initTargetPos.z());

            }

            
          }

          if ( distanceFromOrigin.x() > 0 && distanceFromOrigin.z() > 0 )
          {


            quadrant = 4;
            cout << "The quadrant = " << quadrant << endl ; 
            cout << "actionNumber = " << actionNumber << endl; 
            // if (!flag1)
            // {
            //    cout << "Flag1 = " << flag << endl;

            if ( (actionNumber == 4) || (actionNumber == 5) )
            {
               cout << "This is right" << endl;
               subgoal1.set(initTargetPos.x(), initTargetPos.y(), initTargetPos.z());
              
            }

            else{

               cout << "It will be moved to other quadrant" << endl; 
               subgoal1.set( 50, 1, 160);
               subgoal2.set( -120, 1, 160 );

            }
            // }
            // if (!flag2)
            // {
            //    cout << "It entered the Flag2 = " << flag << endl;
               
            // }
          }

          while ( !flag1 && flag2 )
          {
                 // cout << "This is flag1 loop is " << endl; 

                 cout << "Subgoal 1 in  " << quadrant << "is " << subgoal1.x() << " , " << subgoal1.z() << endl;
                 box->getPosition(targetIntermediatePos);

                 differenceInPos.x(subgoal1.x()-targetIntermediatePos.x());
                 differenceInPos.y(subgoal1.y()-targetIntermediatePos.y());
                 differenceInPos.z(subgoal1.z()-targetIntermediatePos.z());


                  double* ptr2 = controlPosition(subgoal1, targetIntermediatePos, 3.0,0.4, 1.2);
                  box->setLinearVelocity( ptr2[0] , 0, ptr2[1]);

                  if ( abs(differenceInPos.x()) < 0.3 && abs(differenceInPos.z()) < 0.3 )
                  {

                     if (quadrant == 3 && actionNumber!=4 && actionNumber!=5 )
                     {
                          isTargetPositionReset = true;
                          if (myfile.is_open())
                          {
                              myfile << targetIntermediatePos.x() << " , " << targetIntermediatePos.z() << " , " ;
                          }
                     }

                     if (quadrant == 4 && ( actionNumber==4 || actionNumber== 5 ) )
                     {
                          isTargetPositionReset = true;
                          if (myfile.is_open())
                          {
                              myfile << targetIntermediatePos.x() << " , " << targetIntermediatePos.z() << " , " ;
                          }
                     }



                      // LOG_MSG((" Target reset at : %f %f %f ", targetPos.x(), targetPos.y(), targetPos.z() ));
                      LOG_MSG (("The difference in target position is: %f, %f, %f ", differenceInPos.x(), differenceInPos.y(), differenceInPos.z() )) ; 
                      differenceInPos.set(0,0,0);
                      currentVelocity.set(0,0,0);
                      flag1=1;
                      flag2=0;
                     
                          

                 }
          }

          while ( !flag2 && flag1 && !isTargetPositionReset )
          {

                 cout << "Subgoal 2 in  " << quadrant << " is " << subgoal2.x() << " , " << subgoal2.z() << endl;
                 box->getPosition(targetIntermediatePos);

                 differenceInPos.x(subgoal2.x()-targetIntermediatePos.x());
                 differenceInPos.y(subgoal2.y()-targetIntermediatePos.y());
                 differenceInPos.z(subgoal2.z()-targetIntermediatePos.z());


                  double* ptr2 = controlPosition(subgoal2, targetIntermediatePos, 3.0,0.4, 1.2);
                  box->setLinearVelocity( ptr2[0] , 0, ptr2[1]);

                  if ( abs(differenceInPos.x()) < 0.3 && abs(differenceInPos.z()) < 0.3 )
                  {

                      // LOG_MSG((" Target reset at : %f %f %f ", targetIntermediatePos.x(), targetIntermediatePos.y(), targetIntermediatePos.z() ));
                      LOG_MSG (("The difference in target position is: %f, %f, %f ", differenceInPos.x(), differenceInPos.y(), differenceInPos.z() )) ; 
                      differenceInPos.set(0,0,0);
                      currentVelocity.set(0,0,0);
                      flag1=0;
                      flag2=1;

                  }
          
          }



      


          // differenceInPos.x(initTargetPos.x() - targetPos.x());
          // differenceInPos.y(initTargetPos.y() - targetPos.y());
          // differenceInPos.z(initTargetPos.z() - targetPos.z());
          // // LOG_MSG (("The difference in position is: %f, %f, %f ", differenceInPos.x(), differenceInPos.y(), differenceInPos.z() )) ;   

          // box->getLinearVelocity(currentVelocity);  
          // // LOG_MSG (("The target currentVelocity is: %f, %f, %f ", currentVelocity.x(), currentVelocity.y(), currentVelocity.z() )) ; 
          
          // // double* ptr2 = controlPosition(initTargetPos, targetPos, 1.0,0.0, 0.7);
          // double* ptr2 = controlPosition(initTargetPos, targetPos, 3.0,0.4, 1.2);
          // box->setLinearVelocity( ptr2[0] , 0, ptr2[1]);
        


          // if ( abs(differenceInPos.x()) < 0.3 && abs(differenceInPos.z()) < 0.3 )
          //   {

          //     // LOG_MSG((" Target reset at : %f %f %f ", targetPos.x(), targetPos.y(), targetPos.z() ));
          //     LOG_MSG (("The difference in target position is: %f, %f, %f ", differenceInPos.x(), differenceInPos.y(), differenceInPos.z() )) ; 
          //     isTargetPositionReset = true;
          //     differenceInPos.set(0,0,0);
          //     currentVelocity.set(0,0,0);

          //     if (myfile.is_open())
          //     {
          //         myfile << targetPos.x() << " , " << targetPos.z() << " , " ;
          //     }

          //   }




        }


         
         if ( simulationCount ==  totalSimulations  )
          {
            // cout << "The simulation process is terminated by the user" << endl;

            myfile.close();
            totalTime=evt.time();
            cout << "the total time (seconds) taken in these  " << simulationCount << " simulations is " << totalTime << endl;
            cout << "Thus average time taken by one simulation =  " << (totalTime / simulationCount) << endl; 
            exit(0);
          }

      
        Colli = false;
        flag=1;

        // If you wan't simulation to run continously please uncomment the following line. Otherwise only one manipulation will be performed.

         
        if (singleSimulation)
        {
          myfile.close();
          cout << "the total time (seconds) taken in the simulation " << totalTime<< endl;
          exit(0);     
        }
      

     }


  }

  return 0.000001;
      
} 

std::string msg = " ";

void MyController::onRecvMsg(RecvMsgEvent &evt)
{

}


void MyController::onCollision(CollisionEvent &evt) { 

  
  SimObj *tool = getObj("StickTool");
  SimObj *box = getObj("box_001");
  Vector3d targetStopPos;


  if (Colli == false) {  

  
    // const vector<string> & wname  = evt.getWith();       // Get a name of the other  
    // const vector<string> & wparts = evt.getWithParts();  // Get a parts name of the other's collision point  
    // const vector<string> & mparts = evt.getMyParts();    // Get a parts of collision point of myself  
  
    // for(int i = 0; i < wname.size(); i++)  
    //   {  
    //   // Print the name of the other  
    //   LOG_MSG(("\"%s\"", wname[i].c_str()));  
    //   LOG_MSG(("\"%s\"", wparts[i].c_str()));  
    //   LOG_MSG(("\"%s\"", mparts[i].c_str()));  

      // LOG_MSG(("\"%s\"", myParts));  
      // SimObj *obj = getObj(myname());  

      // cout << "i= " << i << endl; 
      Colli = true;  

      // The simulation count should be stored as "number of observation"

      // box->getLinearVelocity(velocityOfTarget);
      // netVelocityTarget=( pow(velocityOfTarget.x(),2) + pow(velocityOfTarget.y(), 2) + pow(velocityOfTarget.z(), 2 ) );
      // netVelocityTarget = sqrt(netVelocityTarget);

      // if ( Colli && netVelocityTarget < 0.1 )
      // {
      //   double* ptr2 = controlPosition(initTargetPos, targetPos, 3.0,0.4, 1.2);
      //   box->setLinearVelocity( ptr2[0] , 0, ptr2[1]);
      // }
      

      simulationCount++;
      cout << "Simulation count is " << simulationCount << endl;

      previousTime = currentTime;
      currentTime  = evt.time();
      timeDiff = currentTime - previousTime;
      cout << "Time difference is " << timeDiff << endl;

      if (timeDiff < 3.0 )
      {
        // Vector3d safePos(-480,1,480);
        // double* ptr2 = controlPosition(safePos, targetPos, 3.0,0.4, 1.2);
        // box->setLinearVelocity( ptr2[0] , 0, ptr2[1]);
        box->getPosition(targetStopPos);
        myfile << targetStopPos.x() << " ,  " << targetStopPos.z();
        
        simulationCount--;

        cout << "Modified Simulation count is " << simulationCount << endl;
      }

      

      first=false;
      isToolPositionReset=false;
      isTargetPositionReset=false;
      isTargetAtSafePos= false;

      isToolRotationReset=false;
      isTargetRotationReset=false;
      
      tool->getLinearVelocity(velocityOfTool);
      //This parameter shall be stored for learning affordances using Bayesian Networks !
      // LOG_MSG(("Velcoity of tool onCollission is : %f %f %f ", velocityOfTool.x(), velocityOfTool.y(), velocityOfTool.z() ));

      box->getLinearVelocity(velocityOfTarget);
      // This parameter shall be stored for learning affordances using Bayesian Networks !
      // LOG_MSG(("Velcoity of object onCollission is : %f %f %f ", velocityOfTarget.x(), velocityOfTarget.y(), velocityOfTarget.z() ));

      if (myfile.is_open())

      {

            if( ( ( (manipulationsPerSample * 1) < simulationCount ) && (simulationCount <= (manipulationsPerSample * 2 ) )   ) || 
              ( ( (manipulationsPerSample * 3) < simulationCount ) && (simulationCount <= (manipulationsPerSample * 4 ) )   )   ||
              ( ( (manipulationsPerSample * 9) < simulationCount ) && (simulationCount <= (manipulationsPerSample * 10 ) )   ) )
            {
               myfile << 0.0 << " , " << 0.0 << " , " << 0.0 << " , " << 0.0 << " , " ;
            }

            else{
                  myfile << velocityOfTool.x() << " , " << velocityOfTool.z() << " , " << velocityOfTarget.x() << " , " << velocityOfTarget.z() << " , " ;
             }

            // myfile << velocityOfTool.x() << " , " << velocityOfTool.z() << " , " << velocityOfTarget.x() << " , " << velocityOfTarget.z() << " , " ;
      }




    

    

      if (Colli)
      {
          
          // This code adds the random compoent in velocity of tool, so that for each manipulation trial, the tool is applied a different velocity

   
          if (isVelocityinXAllowed)
          {
             v_x = rand() % xVelocityVariance ;
          }

          if (isVelocityinZAllowed)
          {
             v_z = rand() % zVelocityVariance;

          }
          
          velocityOnTool.set( firstVelocityOnTool.x() + v_x, 0, firstVelocityOnTool.z() + v_z );   

          double f_x;
          double f_z;
          double r;

          r = ((double) rand() / (RAND_MAX)) ;


          if (isForceinXAllowed)
          {
            // f_x = rand() % int(xForceVariance); // when force in digaonal direction does get varried: 
            
            f_x = r * int(xForceVariance);

            cout << "r = " << r << endl; 
            cout << "f_x = " << f_x << endl;
         
          }

          if(isForceinZAllowed)
          {
            // f_z = rand() % int(zForceVariance); // when force in digaonal direction does get varried: 
            f_z = r * int(zForceVariance);
            cout << "f_z = " << f_z << endl;

          } 



          forceOnTool.set( firstForceOnTool.x() + f_x, 0, firstForceOnTool.z() + f_z ); // when force in digaonal direction gets varried

       



          setPoint_X = velocityOnTool.x();  // this is the desired velocity in X
          setPoint_Z=  velocityOnTool.z();  // this is the desired velocity in Z

          refVal_X = setPoint_X;    // this is the actually applied feedback velocity in X
          refVal_Z = setPoint_Z;    // this is the actually applied feedback velocity in Z


      }






  }
  

}


extern "C" Controller * createController() {  
  return new MyController;  
}  


