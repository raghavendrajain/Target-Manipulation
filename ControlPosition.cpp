#include "ControllerEvent.h"  
#include "Controller.h"  
#include "Logger.h"  
#include <iostream>
#include <fstream>
#include <iomanip>
#include <boost/python.hpp>
#include <Python.h>
#include <dlfcn.h>

using namespace std;

namespace py = boost::python; // create namespace variable for boost::python
using namespace std;

std::string parse_python_exception(); // functional declaration for exception handling

template <typename T> string tostr(const T& t) { ostringstream os; os<<t; return os.str(); } // template to convert double variables to string




double* controlPosition(Vector3d goalPos, Vector3d currentPos, double K_p, double K_i, double K_d) {  

  double *pointer;
  double P_value[1];
  pointer=P_value;
  double P_Value_X;
  double P_Value_Z;


  // goalPos.set(22,1,93);
  // differenceInPos.set(0,0,0);
 
  dlopen("libpython2.7.so", RTLD_LAZY | RTLD_GLOBAL); 
  Py_Initialize();  //initialization of the python interpreter
  try
    {
      // load the main module
       py::object main_module = py::import("__main__");

      // // load the dictionary object out of the main module to create a blank canvas on which python variables and functions can be executed. 
       py::object main_namespace = main_module.attr("__dict__");

       main_module.attr("K_p") = K_p; 
       main_module.attr("K_i") = K_i; 
       main_module.attr("K_d") = K_d; 


       main_module.attr("goalPos") = "[" + tostr(goalPos.x())+" , "+ tostr(goalPos.y())+ " , " + tostr(goalPos.z()) + "]";
       main_module.attr("currentPos") = "[" + tostr(currentPos.x())+" , "+ tostr(currentPos.y())+ " , " + tostr(currentPos.z()) + "]";
       py::exec("import ast", main_namespace);
       py::exec("goalPos = ast.literal_eval(goalPos)", main_namespace);
       // py::exec("print goalPos", main_namespace);
       py::exec("currentPos = ast.literal_eval(currentPos)", main_namespace);
       // py::exec("print currentPos", main_namespace);
       py::exec("import PID as pid", main_namespace);
       py::exec("p_x=pid.PID(K_p, K_i, K_d)",main_namespace);
 
       py::exec("p_z=pid.PID(K_p, K_i, K_d)",main_namespace);
       py::exec("p_x.setPoint(goalPos[0])", main_namespace);
       py::exec("p_z.setPoint(goalPos[2])", main_namespace);
       // py::exec("print p_z.getPoint()", main_namespace);
       py::exec("pid_x=p_x.update(currentPos[0])", main_namespace);
       py::exec("pid_z=p_z.update(currentPos[2])", main_namespace);
       
       // cout << "Update in x is " << py::extract<double>(main_module.attr("pid_x")) << endl;
       // cout << "Update in z is " << py::extract<double>(main_module.attr("pid_z")) << endl;
       P_Value_X = py::extract<double>(main_module.attr("pid_x"));
       P_Value_Z = py::extract<double>(main_module.attr("pid_z"));

       P_value[0] = P_Value_X;
       P_value[1] = P_Value_Z;

       return pointer;
 
       // tool->setLinearVelocity( P_Value_X , 0, P_Value_Z );

    }



     catch(boost::python::error_already_set const &){
        // Parse and output the exception
        std::string perror_str = parse_python_exception();
        std::cout << "Error in Python: " << perror_str << std::endl;
    }

    
}  


double* controlRotation(Rotation goalRot, Rotation currentRot, double K_p, double K_i, double K_d) {  

  double *pointerP;
  double R_value[1];
  pointerP=R_value;


  dlopen("libpython2.7.so", RTLD_LAZY | RTLD_GLOBAL); 
  Py_Initialize();  //initialization of the python interpreter
  try
    {
      // load the main module
       py::object main_module = py::import("__main__");

      // // load the dictionary object out of the main module to create a blank canvas on which python variables and functions can be executed. 
       py::object main_namespace = main_module.attr("__dict__");

       main_module.attr("K_p") = K_p; 
       main_module.attr("K_i") = K_i; 
       main_module.attr("K_d") = K_d; 


       main_module.attr("goalRot") = "[" + tostr(goalRot.qw())+" , "+ tostr(goalRot.qx())+" , "+ tostr(goalRot.qy())+ " , " + tostr(goalRot.qz()) + "]";
       main_module.attr("currentRot") = "[" + tostr(currentRot.qw())+" , "+ tostr(currentRot.qx())+" , "+ tostr(currentRot.qy())+ " , " + tostr(currentRot.qz()) + "]";

       // py::exec("print goalRot", main_namespace);
       // py::exec("print currentRot", main_namespace);
       py::exec("import ast", main_namespace);
       py::exec("goalRot = ast.literal_eval(goalRot)", main_namespace);

       py::exec("import ast", main_namespace);
       py::exec("currentRot = ast.literal_eval(currentRot)", main_namespace);


       py::exec("import transformations as T", main_namespace);
       py::exec("euler_goal= T.euler_from_quaternion(goalRot)", main_namespace);
       // cout << "The Goal is" << endl;
       // py::exec("print  euler_goal", main_namespace);
       py::exec("euler_current= T.euler_from_quaternion(currentRot)", main_namespace);
       // cout << "The Current is" << endl;
       // py::exec("print euler_current", main_namespace);
       py::exec("diff= euler_current[1]-euler_goal[1]", main_namespace);
       double difference = py::extract<double>(main_module.attr("diff"));
       // cout << "difference is  " << difference <<  endl;

        py::exec("import PID as pid", main_namespace);
        py::exec("p_y=pid.PID(K_p, K_i, K_d)",main_namespace);
        py::exec("p_y.setPoint(euler_goal[1])", main_namespace);

        py::exec("pid_y=p_y.update(euler_current[1])", main_namespace);

         
        // cout << "Update in y is " << py::extract<double>(main_module.attr("pid_y")) << endl;
   
        double P_Value_Y = py::extract<double>(main_module.attr("pid_y"));
       
        // return P_Value_Y; 

        R_value[0] = P_Value_Y;
        R_value[1] = difference;

        // R_value[0] = K_p;
        // R_value[1] = K_d;

        return pointerP;

       // return difference; 

  }

  catch(boost::python::error_already_set const &){
        // Parse and output the exception
        std::string perror_str = parse_python_exception();
        std::cout << "Error in Python: " << perror_str << std::endl;
    }

}



std::string parse_python_exception(){
    PyObject *type_ptr = NULL, *value_ptr = NULL, *traceback_ptr = NULL;
    // Fetch the exception info from the Python C API
    PyErr_Fetch(&type_ptr, &value_ptr, &traceback_ptr);

    // Fallback error
    std::string ret("Unfetchable Python error");
    // If the fetch got a type pointer, parse the type into the exception string
    if(type_ptr != NULL){
        py::handle<> h_type(type_ptr);
        py::str type_pstr(h_type);
        // Extract the string from the boost::python object
        py::extract<std::string> e_type_pstr(type_pstr);
        // If a valid string extraction is available, use it 
        //  otherwise use fallback
        if(e_type_pstr.check())
            ret = e_type_pstr();
        else
            ret = "Unknown exception type";
    }
    // Do the same for the exception value (the stringification of the exception)
    if(value_ptr != NULL){
        py::handle<> h_val(value_ptr);
        py::str a(h_val);
        py::extract<std::string> returned(a);
        if(returned.check())
            ret +=  ": " + returned();
        else
            ret += std::string(": Unparseable Python error: ");
    }
    // Parse lines from the traceback using the Python traceback module
    if(traceback_ptr != NULL){
        py::handle<> h_tb(traceback_ptr);
        // Load the traceback module and the format_tb function
        py::object tb(py::import("traceback"));
        py::object fmt_tb(tb.attr("format_tb"));
        // Call format_tb to get a list of traceback strings
        py::object tb_list(fmt_tb(h_tb));
        // Join the traceback strings into a single string
        py::object tb_str(py::str("\n").join(tb_list));
        // Extract the string, check the extraction, and fallback in necessary
        py::extract<std::string> returned(tb_str);
        if(returned.check())
            ret += ": " + returned();
        else
            ret += std::string(": Unparseable Python traceback");
    }
  }
