/**
 * @file /kobuki_core/src/tools/simple_keyop.cpp
 *
 * @brief Tools/utility program to control robot by keyboard.
 *
 * License: BSD
 *   https://raw.githubusercontent.com/kobuki-base/kobuki_core/license/LICENSE
 **/

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <string>
#include <csignal>
#include <termios.h> // for keyboard input
#include <iostream>
#include <fstream>
#include <string>

#include <ecl/command_line.hpp>
#include <ecl/console.hpp>
#include <ecl/geometry.hpp>
#include <ecl/linear_algebra.hpp>
#include <ecl/time.hpp>
#include <ecl/threads.hpp>
#include <ecl/sigslots.hpp>
#include <ecl/exceptions.hpp>
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()


#include "kobuki_core/kobuki.hpp"

/*****************************************************************************
** Classes
*****************************************************************************/

/**
 * @brief Keyboard remote control for our robot core (mobile base).
 *
 */
class KobukiManager
{
public:

  /*********************
   ** C&D
   **********************/
  KobukiManager();
  ~KobukiManager();
  bool init(const std::string & device);

  /*********************
   ** Callbacks
   **********************/
  void processStreamData();

  /*********************
   ** Accessor
   **********************/
  const ecl::linear_algebra::Vector3d& getPose() { return pose; };
  bool isShutdown() { return quit_requested || kobuki.isShutdown(); };
  int getSerialPort();
  void closeFile();

private:
  double vx, wz;
  bool pickup_mode;
  ecl::linear_algebra::Vector3d pose;
  kobuki::Kobuki kobuki;

  double linear_vel_step, linear_vel_max;
  double angular_vel_step, angular_vel_max;
  std::string name;
  ecl::Slot<> slot_stream_data;
  int serial_port;
  std::ifstream pi_input_file;

  /*********************
   ** Commands
   **********************/
  void resetVelocity();

  /*********************
   ** Debugging
   **********************/
  void relayWarnings(const std::string& message);
  void relayErrors(const std::string& message);

  /*********************
   ** Keylogging
   **********************/
  void piInput();
  void processPiInput(int i);
  void restoreTerminal();
  bool quit_requested;
  int key_file_descriptor;
  struct termios original_terminal_state;
  ecl::Thread thread;
  ecl::Mutex mutex;
};

/*****************************************************************************
 ** Implementation
 *****************************************************************************/

/**
 * @brief Default constructor, needs initialisation.
 */
KobukiManager::KobukiManager() :
  vx(0.0), wz(0.0),
  pickup_mode(false),
  linear_vel_step(0.05),
  linear_vel_max(1.0),
  angular_vel_step(0.33),
  angular_vel_max(6.6),
  slot_stream_data(&KobukiManager::processStreamData, *this),
  quit_requested(false),
  key_file_descriptor(0)
{
  tcgetattr(key_file_descriptor, &original_terminal_state); // get terminal properties
}

KobukiManager::~KobukiManager()
{
  kobuki.setBaseControl(0,0); // linear_velocity, angular_velocity in (m/s), (rad/s)
  kobuki.disable();
  tcsetattr(key_file_descriptor, TCSANOW, &original_terminal_state);
}

/**
 * @brief Initialises the node.
 */
bool KobukiManager::init(const std::string & device)
{

  /*********************
   ** Velocities
   **********************/
  vx = 0.0;
  wz = 0.0;
  pickup_mode = false;

  /*********************
   ** Kobuki
   **********************/
  kobuki::Parameters parameters;
  parameters.sigslots_namespace = "/kobuki";
  parameters.device_port = device;
  parameters.enable_acceleration_limiter = true;

  kobuki.init(parameters);
  kobuki.enable();
  slot_stream_data.connect("/kobuki/stream_data");

  /*********************
   ** Wait for connection
   **********************/
  //thread.start(&KobukiManager::piInputLoop, *this);
  pi_input_file.open("serial/kobuki-input.txt");

  serial_port = open("/dev/ttyUSB0", O_RDWR);
  // Check for errors
  if (serial_port < 0) {
      //printf("Error %i from open: %s\n", errno, strerror(errno));
  } 
  return true;
}

/*****************************************************************************
 ** Implementation [Keyboard]
 *****************************************************************************/

int KobukiManager::getSerialPort() {
  return serial_port;
}

void KobukiManager::closeFile() {
  pi_input_file.close();
}

void KobukiManager::piInput()
{
  std::string file_input;
  if (pi_input_file.is_open()) {
    pi_input_file >> file_input;
    int pi_input = stoi(file_input);
    std::cout << "read" << pi_input << "from file\n";
    processPiInput(pi_input);
    pi_input_file.clear();
    pi_input_file.seekg(0, pi_input_file.beg);
    
  } else {
    processPiInput(20);
  }
  //int pi_input = 50;
  //processPiInput(pi_input);
  
} 

void KobukiManager::processPiInput(int i)
{
  // i [1, 100] 
  // i = 50 -> go straight
  // i > 50 -> go right
  // i < 50 -> go left
  if (pickup_mode || true) {
    if (i == 50) {
    	vx = 0.1;
    	wz = 0.0;
    } else if (i == 2) {
    	pickup_mode = false;
    	//move back to original position
    } else if (i >= 1 && i <= 100) {
    // TODO: fix this
    	vx = 0.3;
    	//wz = (i - 50)*0.01;
      wz = 0.0;
    }
 
  } else {
    if (i == 0) {
    	pickup_mode = true;
    	// record initial position
    	//const ecl::linear_algebra::Vector3d& pose = kobuki_manager.getPose();
    } 
  }
}


/*****************************************************************************
 ** Implementation [Commands]
 *****************************************************************************/

void KobukiManager::resetVelocity()
{
  mutex.lock();
  vx = 0.0;
  wz = 0.0;
  mutex.unlock();
}

void KobukiManager::processStreamData() {
  ecl::linear_algebra::Vector3d pose_update;
  ecl::linear_algebra::Vector3d pose_update_rates;
  kobuki.updateOdometry(pose_update, pose_update_rates);
  ecl::concatenate_poses(pose, pose_update);
  // TODO(daniel.stonier): this needs a mutex
  // This callback triggers in Kobuki's thread, however
  // vx, wz are updated in the keyboard input thread.
  piInput();
  kobuki.setBaseControl(vx, wz);
}

/*****************************************************************************
** Signal Handler
*****************************************************************************/

bool signal_shutdown_requested = false;
void signalHandler(int /* signum */) {
  signal_shutdown_requested = true;
}

/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char** argv)
{
  ecl::CmdLine cmd_line("simple_keyop program", ' ', "0.3");
  ecl::UnlabeledValueArg<std::string> device_port("device_port", "Path to device file of serial port to open, connected to the kobuki", false, "/dev/kobuki", "string");
  cmd_line.add(device_port);
  cmd_line.parse(argc, argv);

  signal(SIGINT, signalHandler);

  std::cout << ecl::bold << "\nDrives Kobuki based off inputs from raspberry pi.\n" << ecl::reset << std::endl;
  KobukiManager kobuki_manager;
  kobuki_manager.init(device_port.getValue());

  struct termios tty;
  if(tcgetattr(kobuki_manager.getSerialPort(), &tty) != 0) {
      //printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
  }
  tty.c_cflag &= ~PARENB; 

  tty.c_cflag &= ~CSTOPB; 

  tty.c_cflag &= ~CSIZE; 
  tty.c_cflag |= CS8; 

  tty.c_cflag &= ~CRTSCTS; 

  tty.c_cflag |= CREAD | CLOCAL;

  tty.c_lflag &= ~ICANON; 
  
  tty.c_lflag &= ~ECHO; // Disable echo
  tty.c_lflag &= ~ECHOE; // Disable erasure
  tty.c_lflag &= ~ECHONL; // Disable new-line echo

  tty.c_lflag &= ~ISIG;

  tty.c_iflag &= ~(IXON | IXOFF | IXANY);

  tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); 

  tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
  tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed

  tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
  tty.c_cc[VMIN] = 0;

  cfsetispeed(&tty, B115200);
  cfsetospeed(&tty, B115200);

  if (tcsetattr(kobuki_manager.getSerialPort(), TCSANOW, &tty) != 0) {
    //printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
    //std::cout << "Error" << errno << "from tcsetattr:" << strerror(errno);
  }



  ecl::Sleep sleep_one_second(1);
  try {
    while (!signal_shutdown_requested && !kobuki_manager.isShutdown()){
      sleep_one_second();
      // const ecl::linear_algebra::Vector3d& pose = kobuki_manager.getPose();
      // std::cout << ecl::green;
      // std::cout << "current pose: [x: " << pose[0] << ", y: " << pose[1] << ", heading: " << pose[2] << "]" << std::endl;
      // std::cout << ecl::reset;
    }
  } catch ( ecl::StandardException &e ) {
    std::cout << e.what();
  }
  close(kobuki_manager.getSerialPort());
  kobuki_manager.closeFile();
  return 0;
}
