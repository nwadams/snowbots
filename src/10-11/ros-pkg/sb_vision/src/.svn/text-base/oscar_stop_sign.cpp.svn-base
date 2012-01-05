#include <boost/program_options.hpp>
#include <cmath>
#include <cstdlib>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <fstream>
#include <iostream>
#include <ostream>
#include <ros/ros.h>
#include <string>
#include <vector>

#include <sb_config/config_file.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float64.h>

using std::string;
using std::cout;
using std::endl;
using std::ostream;
using std::vector;
using namespace cv;
namespace po = boost::program_options;

/* Global constants */
const string NODE_NAME = "visual_force_nav";
const string CMD_TOPIC = "visual_force_nav";
const string SS_TOPIC = "stopsign";
const int MSG_QUEUE_SIZE = 20;

/* Global variables */
int start_row = 40;
int stop_row = 40;
double framerate = 10.0;
int process_every = 1;
int ss_quality = 190;
ros::Publisher cmd_pub;
ros::Publisher ss_pub;
geometry_msgs::Vector3 output_msg;
std_msgs::Float64 ss_msg;


uchar max(uchar first, uchar second)
{
  return saturate_cast<uchar>( first > second ? first : second );
}

void debug_vector(Mat img, const geometry_msgs::Vector3& vec)
{
  Point src(img.cols / 2, img.rows / 2);
  Point end(src.x + vec.y, src.y + vec.x);
  Scalar color(255, 255, 255);
  int thickness = 4;
  line(img, src, end, color, thickness);
}

void debug_circle(Mat img, int row, int col)
{
  Point center(col, row);
  Scalar color(255, 255, 255);
  int radius = 10;
  circle(img, center, radius, color);
}

void do_channel(Mat channel)
{
#ifdef DEBUG
  imshow("red", channel);
#endif
  int base_row = channel.rows - start_row - 1;
  int base_col = channel.cols / 2;
  int base_val = channel.at<uchar>(base_row, base_col);
  double x_total = 0.0, y_total = 0.0;
  int row_num = base_row;
  int row_min = 0;
  while ( row_num >= stop_row )
  {
    uchar* row_ptr = channel.ptr<uchar>(row_num);
    row_min = max(row_min, abs(int(row_ptr[base_col]) - base_val));
    int col_min = row_min;
    int col_num = base_col - 1;
    while ( col_num >= 0 )
    {
      int current_val = row_ptr[col_num];
      col_min = max(col_min, abs(current_val - base_val));
      row_ptr[col_num] = col_min; // paint over the original pixel
      int x_pos = base_row - row_num;
      int y_pos = base_col - col_num;
      double dist = sqrt( double(x_pos * x_pos) + double(y_pos * y_pos) );
      double force = double(col_min) / dist;
      x_total += force * x_pos;
      y_total += force * y_pos;
      col_num = col_num - 1;
    }

    col_min = row_min;
    col_num = base_col + 1;
    while ( col_num < channel.cols )
    {
      int current_val = row_ptr[col_num];
      col_min = max(col_min, abs(current_val - base_val));
      row_ptr[col_num] = col_min; // paint over the original pixel
      int x_pos = base_row - row_num;
      int y_pos = base_col - col_num;
      double dist = sqrt( double(x_pos * x_pos) + double(y_pos * y_pos) );
      double force = double(col_min) / dist;
      x_total += force * x_pos;
      y_total += force * y_pos;
      col_num = col_num + 1;
    }

    row_num = row_num - 1;
  }
  int num_pixels = channel.rows * channel.cols;
  output_msg.x = x_total / num_pixels;
  output_msg.y = y_total / num_pixels;
  output_msg.z = 0.0;
#ifdef DEBUG
  debug_vector(channel, output_msg);
  imshow("output", channel);
#endif
  cout << "force vector is <" << output_msg.x << ", "
                              << output_msg.y << ">" << endl;
  cmd_pub.publish(output_msg);
}


void do_frame(Mat img)
{
#ifdef DEBUG
  imshow("input", img);
#endif
  Mat small;
  resize(img, small, Size(0,0), 0.4, 0.4);
  vector<Mat> planes;
  split(small, planes);
  do_channel(planes[2]);
}

void do_ss_frame(Mat img)
{
#ifdef DEBUG
  imshow("input", img);
#endif
  Mat small;
  resize(img, small, Size(0,0), 0.4, 0.4);
  vector<Mat> channels;
  split(small, channels);
  Mat blue = channels[0];
  Mat green = channels[1];
  Mat red = channels[2];
  const static long max = 255 * (red.rows / 2) * (red.cols / 2);
  long total = 0;
  for ( int rownum = red.rows / 2; rownum > 0; rownum-- )
  {
    uchar* blue_row = blue.ptr<uchar>(rownum);
    uchar* green_row = green.ptr<uchar>(rownum);
    uchar* red_row = red.ptr<uchar>(rownum);
    for ( int colnum = red.cols / 2; colnum < red.cols; colnum++ )
    {
      int cell_val = red_row[colnum] - 
        (blue_row[colnum] + green_row[colnum]);
      cell_val = ((255 * 2) + cell_val) / 3;
      if ( cell_val > ss_quality ) {
        total += cell_val;
        red_row[colnum] = cell_val;
      }
      else {
        red_row[colnum] = 0;
      }
    }
  }
#ifdef DEBUG
  imshow("stopsign", red);
#endif
  ss_msg.data = double(total) / double(max);
  cout << "Stopsign is " << ss_msg.data << endl;
  ss_pub.publish(ss_msg);
}

void do_video(VideoCapture capture)
{
  double fps = capture.get(CV_CAP_PROP_FPS);
  cout << "framerate is " << fps << endl;
  int waitframe;
  int frame_no = 0;
  if ( fps > 0 ) {
    waitframe = int(1000.0 / fps);
  }
  else {
    waitframe = int(1000.0 / (framerate * process_every));
  }
  cout << "waitframe is " << waitframe << " ms" << endl;
  Mat frame;
  while ( ros::ok() )
  {
    capture >> frame;
    if (! frame.data ) {
      ROS_FATAL("Unable to capture a frame");
      return;
    }
    if ( frame_no % process_every == 0 ) {
      do_frame(frame);
    }
    else if ( frame_no % process_every == 1 ) {
      do_ss_frame(frame);
    }
    char key = waitKey(waitframe);
    if ( key == 'q' || key == ' ' ) {
      ros::shutdown();
    }
    frame_no ++;
  }
}


void do_image(Mat img)
{
  do_frame(img);
  waitKey(0);
}

void load_config(const string& filename)
{
  // Describe the valid options
  po::options_description desc;
  desc.add_options()
  ("ian_nav.start_row", po::value<int>( &start_row ))
  ("ian_nav.stop_row", po::value<int>( &stop_row ))
  ("ian_nav.framerate", po::value<double>( &framerate ))
  ("ian_nav.process_every", po::value<int>( &process_every ))
  ("oscar.stopsign.quality", po::value<int>( &ss_quality ));

  // Create a place to store the option values
  po::variables_map vm;
  // Load the options into the map
  std::ifstream filestream(filename.c_str());
  po::store(po::parse_config_file(filestream, desc, true), vm);
  po::notify(vm);
}

int main(int argc, char** argv)
{
  /* Initialize node */
  ROS_INFO("Starting %s", NODE_NAME.c_str());
  ros::init(argc, argv, NODE_NAME);
  ros::NodeHandle node;

  /* Load configuration file */
  std::string cfgfile;
  if (! sb_config::findConfigFile(argc, argv, cfgfile) ) {
    ROS_FATAL("Can't find configuration file.");
    ros::shutdown();
  }
  ROS_INFO("Loading configuration in %s", cfgfile.c_str());
  load_config(cfgfile);
  
  /* Create publisher and subscribers */
  cmd_pub = node.advertise<geometry_msgs::Vector3>(
      CMD_TOPIC, MSG_QUEUE_SIZE
  );
  //ss_pub = node.advertise<std_msgs::Float64>(
    //  SS_TOPIC, MSG_QUEUE_SIZE//TODO mark publish this
  //);


#ifdef DEBUG
  namedWindow("input");
  namedWindow("red");
  namedWindow("stopsign");
  namedWindow("output");
#endif
  if ( argc > 2 ) {
    ROS_INFO("Doing image %s", argv[2]);
    Mat img = imread(argv[2]);
    if (! img.data ) {
      ROS_FATAL("Unable to load image %s", argv[2]);
      return 1;
    }
    do_image(img);
  }
  else if ( argc > 1 ) {
    ROS_INFO("Doing video file %s", argv[1]);
    VideoCapture capture(argv[1]);
    if (! capture.isOpened() ) {
      ROS_FATAL("Unable to open video file %s", argv[1]);
      return 1;
    }
    do_video(capture);
  }
  else {
    ROS_INFO("Doing camera capture");
    for ( int cam_id = 5; cam_id >= 0; cam_id-- ) {
      VideoCapture capture(cam_id);
      if ( capture.isOpened() ) {
        do_video(capture);
        return 0;
      }
      else {
        ROS_WARN("Unable to open camera %i", cam_id);
      }
    }
    ROS_FATAL("Unable to open any cameras");
  }
  ROS_INFO("Shutting down %s", NODE_NAME.c_str());
}

