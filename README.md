#  Coding standard AIAA:

  -  Branch name: Ex: BMP581-Issue#1
  -  camelCase() function name
  -  m_memberVariables
  -  Comment on code
  -  Write as few line as possible
  -  Avoid Deep nesting (avoid double loop function that increases time complexity)
  -  Avoid long lines
  -  Explicit naming (no i,j,etc... Ex: time)
  -  Unit test code (TBD)
  -  ReadMe for your code
  -  Error handling. Print out to serial monitor sensor/state error
  -  Automate if necessary
<br><br><br>
# Setting up test dependencies

## Overview
  - Due to highly complexity of the state machine for many safety reason and filtering to prevent invalid data from sensors, GTest will be used to ensure failure cases are caught in time and prevent effectively during flight operation.
  - The folder **test_components** contains Google Test Framework version 1.17.0 on 04/30/2025
## Setup Example
  - Since we are using ESP-IDF framework, we can use the ESP-IDF terminal to run our necessary commands
  - `cd` into the example folder
  - `mkdir build` to create a build folder 
  - `cd` into build folder and do `cmake ..`
  - You should see something like : <br>
      -- Looking for pthread_create in pthread - not found <br>
      -- Found Threads: TRUE <br>
      -- Configuring done (8.2s) <br>
      -- Generating done (0.2s) <br>
  - Do `cmake --build .` This will create an executable in **build/Debug/example_test.exe**
  - Do `.\Debug\example_test.exe` to run test
  - Enjoy the green (or red) lines from your test!


