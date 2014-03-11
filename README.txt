The code enables robot to manipulate target object several times using different samples, simulatenouly. 
A sample means using a combination of Action and Functional feature. 
A total of 3 samples are used in this code. 


The visualization of tool and its dynamics shape is different. That is the problem.
Otherwise, its a SUCCESS too!


--------------------------------
How to run the code ?
--------------------------------

1. The code uses the PID Controller written in Python, so please install and configure boost.python with SIGVerse. 
The tutorial to do that is written here: http://www.sigverse.org/wiki/en/index.php?Embedding%20Python%20Interpreter

2. Run Makefile. # You may need to edit the Makefile, to point EXTERN_DIR  to your boost installation.

$make


3. Add PYTHONPATH to your current working directory 

Find your current working directory

$pwd

Add PYTHONPATH 

$ export PYTHONPATH=${PYTHONPATH}: /your_path_to_work_directory

4. Load world file into SIGViewer.

$sigserver.sh -w ./ToolsOnTable.xml
