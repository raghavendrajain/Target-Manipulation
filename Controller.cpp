#include "ControllerEvent.h"
#include "Controller.h"
#include "Logger.h"
#include <iostream>
#include "math.h" 
#include <iostream>
#include <fstream>
#include <iomanip>
#include "ControlPosition.cpp" // This controls the position of tool and target using PID.


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


bool multipleSamples = true ; // yes if more than one sample is taken.
int numberOfSamples = 3 ;     // set the number of samples for manipulations. the rules for each sample are set below!
int manipulationsPerSample = 2; // if multiple samples are taken sequentially, the set the number of manipulations per sample.


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

static bool isTargetRotationReset;  // reset the target rotation to the rotation set in XML world file // CURRENTLY: functionality not implemented!
static bool isToolRotationReset;    // reset the target rotation to the rotation set in XML world file // CURRENTLY: functionality not implemented!




static Vector3d velocityOnTool;  // the first manipulation trial is performed using the preset velocity. then it is updated for each subseqent trial.

static int messageCount;
static int onActionCount;

int flag;

static Vector3d pos;
Vector3d initToolPos; // this stores initial position of the tool, used to reset the position when it comes to rest collision with target.
Vector3d toolPos;     // this stores current position of the target, used to reset the position when it comes to rest after being hit by the tool.


Vector3d velocityOfTool;    // gets the velocity of tool.
static int simulationCount=0; // counts how many times, the manipulation trial is performed.
int currentCount=0;
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

Vector3d differenceInPos;
Vector3d currentVelocity; // current velocity of Tool


void MyController::onInit(InitEvent &evt) {

  first=true;


  SimObj *tool = getObj("StickTool");
  SimObj *box = getObj("box_001");
  tool->setMass(5.0); // the same mass is to be used for all experiments to maintain consistency!

  
  // when multiple samples is not taken and  only single simulation is used 
  if (singleSimulation && !multipleSamples)
  {
     firstVelocityOnTool.set(0,0,20);  // for "Contract Arm" action
     
  }

   
  // when multiple samples are not taken, but many simulations of the same sample is taken.  
  if( !singleSimulation && !multipleSamples )

  {

     firstVelocityOnTool.set(0,0,20);  // for "Contract Arm" action

     isVelocityinXAllowed = 0; // for "Contract Arm " action, only Z component of velocity is set. But change it if you want to apply "Push Diagonally" or "Slide Left"
     isVelocityinZAllowed = 1; // for "Contract Arm " action, only Z component of velocity is set

     // xVelocityVariance = 20 ; // change the variance in x conmpont of desired velocity. Only works if "isVelocityinXAllowed" flag is set.
     zVelocityVariance = 20;  // change the variance in Z conmpont of desired velocity. Only works if "isVelocityinZAllowed" flag is set.

     doesTargetPositionReset = 1; // reset the target position to the position set in XML world file 
     doesToolPositionReset = 1;   // reset the tool position to the position set in XML world file   

     isTargetRotationReset = 0;  // reset the target rotation to the rotation set in XML world file // CURRENTLY: functionality not implemented!
     isToolRotationReset = 0;    // reset the target rotation to the rotation set in XML world file // CURRENTLY: functionality not implemented!

  }

  // the value of initTargetPos variable is acquired from XML file in case no multiple samples are used for target manipulation, else they are 
  // used from rules for each sample.
  if (!multipleSamples )
  {
    box->getPosition(initTargetPos);
  }



  velocityOnTool = firstVelocityOnTool;

  tool->setLinearVelocity(velocityOnTool.x(), velocityOnTool.y(), velocityOnTool.z() );

  setPoint_X = velocityOnTool.x();  // this is the desired velocity in X
  setPoint_Z=  velocityOnTool.z();  // this is the desired velocity in Z

  refVal_X = setPoint_X;    // this is the actually applied feedback velocity in X
  refVal_Z = setPoint_Z;    // this is the actually applied feedback velocity in Z

 
  Colli = false; 
  flag=1;
  isToolPositionReset=true;
  isTargetPositionReset=true;

  differenceInPos.set(0,0,0);
  tool->getPosition(initToolPos);


  // tool->getRotation(rotTool); 
  // box->getRotation(rotObj);

  // This parameter shall be stored for learning affordances using Bayesian Networks !
  LOG_MSG((" Initial Target position onInit() ))is : %f %f %f ", initTargetPos.x(), initTargetPos.y(), initTargetPos.z() ));

  myfile.flush();


  if (myfile.is_open())
  {
     myfile << "InitialToolPosX" << " , " << "InitialToolPosZ" << " , " << "InitialTargetPosX" << " , " << "InitialTargetPosZ" << " , ";
     myfile <<  "xSetVelcoity_tool" << " , " << "zSetVelocity_tool" << " , "; 
     myfile << "xVelAtContact_Tool" << " , " << "zVelAtContact_Tool" << " , " << "xVelAtContact_Target" << " , " << "zVelAtContact_Target" <<  " , ";
     myfile <<  "targetFinalPosX" << " , " << "targetFinalPosZ"  << " , " << "targetDisplacement_X" << " , " << "targetDisplacement_Z"   <<"\n" ; 

     myfile << setprecision(2) << std::fixed;

     myfile << initToolPos.x() << " , "  << initToolPos.z() << " , ";
     myfile << initTargetPos.x() << " , " << initTargetPos.z() << " , " ;

  }


 
}  

// --------------------------------

int sampleCount=0;

bool flag_set_parameters = true;
  
double MyController::onAction(ActionEvent &evt) {  
  //return 1.0;
  SimObj *tool = getObj("StickTool");
  SimObj *box = getObj("box_001");

  cout << "The simulation count in beginning is " << simulationCount << endl;

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
    if  ( (0 <= simulationCount ) && (simulationCount < manipulationsPerSample )  )
    {

          cout << " first loop simulation count " << simulationCount <<  endl; 

          firstVelocityOnTool.set(0,0,20);
          velocityOnTool = firstVelocityOnTool;


          isVelocityinXAllowed = 0; 
          isVelocityinZAllowed = 1;

          // xVelocityVariance = 20 ; // change the variance in x conmpont of desired velocity. Only works if "isVelocityinXAllowed" flag is set.
          zVelocityVariance = 20;  // change the variance in Z conmpont of desired velocity. Only works if "isVelocityinZAllowed" flag is set.

          doesTargetPositionReset = 1; // reset the target position to the position set in XML world file 
          doesToolPositionReset =   1 ;  // reset the tool position to the position set in XML world file   


          initTargetPos.set(5,1,44);
         
       }

       // for slide left arm with vertical part feature 
  
    else  if  ( ( (manipulationsPerSample * 1) <= simulationCount ) && (simulationCount < (manipulationsPerSample * 2 ) )   )  

     {

    
          firstVelocityOnTool.set(-20,0,0);
          velocityOnTool = firstVelocityOnTool;

          isVelocityinXAllowed = 1; 
          isVelocityinXAllowed = 0;

          xVelocityVariance = -20 ; // change the variance in x conmpont of desired velocity. Only works if "isVelocityinXAllowed" flag is set.
          // zVelocityVariance = 20;  // change the variance in Z conmpont of desired velocity. Only works if "isVelocityinZAllowed" flag is set.


          initTargetPos.set(6,1,48);
    
          
    }

        // for pull arm diagonally with corner feature 
    else if ( ( (manipulationsPerSample * 2)  <= simulationCount ) && (simulationCount < (manipulationsPerSample * 3))  ) {


          firstVelocityOnTool.set(-20,0,20);
          velocityOnTool = firstVelocityOnTool;

          isVelocityinXAllowed = 1; 
          isVelocityinXAllowed = 1;
  
          xVelocityVariance = -20 ; // change the variance in x conmpont of desired velocity. Only works if "isVelocityinXAllowed" flag is set.
          zVelocityVariance = 20;  // change the variance in Z conmpont of desired velocity. Only works if "isVelocityinZAllowed" flag is set.

          initTargetPos.set(6,1,48);
  
          
    }

    else {
      cout << "The limit of manipulation samples is reached " << endl;
      exit(0);
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
        if (myfile.is_open())
        {
          myfile << velocityOnTool.x() << " , " <<velocityOnTool.z() << " , "; 
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
      
     tool->setLinearVelocity(velocityOnTool.x(), velocityOnTool.y(), velocityOnTool.z() ); // without applying P-Controller. 
 
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
            myfile << targetPos.x() << " , " << targetPos.z() << " , " ;
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


        // // I want the position of tool to reset, once it has come to rest after hitting the target object. 


        // // Please comment out the following four lines for reseting tool to original position 

        while  ( doesToolPositionReset && !singleSimulation && !isToolPositionReset  && ( simulationCount < totalSimulations) )
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
          
            
            double* ptr1 = controlPosition(initToolPos, toolPos, 1.0,0.0, 0.7);
            tool->setLinearVelocity( ptr1[0] , 0, ptr1[1]);

            if ( abs(differenceInPos.x()) < 0.1 && abs(differenceInPos.z()) < 0.1)
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

        while (doesTargetPositionReset && !singleSimulation && !isTargetPositionReset && isToolPositionReset && ( simulationCount < totalSimulations))
        {

    
          box->getPosition(targetPos);
          // This parameter shall be stored for learning affordances using Bayesian Networks !
          // LOG_MSG((" Target pos is : %f %f %f ", targetPos.x(), targetPos.y(), targetPos.z() ));

           // Storing the reset position into the file. This is not required for learning BN, but just for checks. 



          differenceInPos.x(initTargetPos.x() - targetPos.x());
          differenceInPos.y(initTargetPos.y() - targetPos.y());
          differenceInPos.z(initTargetPos.z() - targetPos.z());
          // LOG_MSG (("The difference in position is: %f, %f, %f ", differenceInPos.x(), differenceInPos.y(), differenceInPos.z() )) ;   

          box->getLinearVelocity(currentVelocity);  
          // LOG_MSG (("The target currentVelocity is: %f, %f, %f ", currentVelocity.x(), currentVelocity.y(), currentVelocity.z() )) ; 
          
            
          double* ptr2 = controlPosition(initTargetPos, targetPos, 1.0,0.0, 0.7);
          box->setLinearVelocity( ptr2[0] , 0, ptr2[1]);

          if ( abs(differenceInPos.x()) < 0.2 && abs(differenceInPos.z()) < 0.2)
            {

              // LOG_MSG((" Target reset at : %f %f %f ", targetPos.x(), targetPos.y(), targetPos.z() ));
              LOG_MSG (("The difference in target position is: %f, %f, %f ", differenceInPos.x(), differenceInPos.y(), differenceInPos.z() )) ; 
              isTargetPositionReset = true;
              differenceInPos.set(0,0,0);
              currentVelocity.set(0,0,0);

              if (myfile.is_open())
              {
                  myfile << targetPos.x() << " , " << targetPos.z() << " , " ;
              }

            }

        }

        if (isToolRotationReset)
        {
           // orientation reset functionality has not been implemented yet!
          

        }

        if (isTargetRotationReset)
        {
            // orientation reset functionality has not been implemented yet!
        }

         
         if ( simulationCount == totalSimulations )
          {
            // cout << "The simulation process is terminated by the user" << endl;

            myfile.close();
            totalTime=evt.time();
            cout << "the total time (seconds) taken in these " << simulationCount << "simulations is " << totalTime << endl;
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
      simulationCount++;
      cout << "Simulation count is " << simulationCount << endl;

      first=false;
      isToolPositionReset=false;
      isTargetPositionReset=false;
      
      tool->getLinearVelocity(velocityOfTool);
      //This parameter shall be stored for learning affordances using Bayesian Networks !
      // LOG_MSG(("Velcoity of tool onCollission is : %f %f %f ", velocityOfTool.x(), velocityOfTool.y(), velocityOfTool.z() ));

      box->getLinearVelocity(velocityOfTarget);
      // This parameter shall be stored for learning affordances using Bayesian Networks !
      // LOG_MSG(("Velcoity of object onCollission is : %f %f %f ", velocityOfTarget.x(), velocityOfTarget.y(), velocityOfTarget.z() ));

      if (myfile.is_open())
      {
            myfile << velocityOfTool.x() << " , " << velocityOfTool.z() << " , " << velocityOfTarget.x() << " , " << velocityOfTarget.z() << " , " ;
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


