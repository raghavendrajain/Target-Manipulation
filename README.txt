The code enables robot to manipulate target object several times using different samples, simulatenouly. 
A sample means using a combination of Action and Functional feature. 
A total of 3 samples are used in this code. 

I implemented my software in SIGVerse which enables target manipulation using different pre-programmed actions and knowledge of functional feartures.
Robot manipulates target using three actions 1: Contract Arm, 2: Slide Left 3: Pull Diagonally with three functional features i) Horizontal part ii) Vertical part iii)corner.

The code enables robot to manipulate target object several times using different samples, simulatenouly. A sample means using a combination of Action and Functional feature. For each sample six manipulation trials are taken. In each manipulation trial, a force of a varying magnitude is applied to the tool.
Position and orientation control of tool and target object is used, after the object comes to rest. 


The visualization of tool and its dynamics shape is different. That is the only problem.

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
