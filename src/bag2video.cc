#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#include <sensor_msgs/CompressedImage.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cstdlib>


#define foreach BOOST_FOREACH
using namespace std;

string codec ("MJPG");


int main (int argc, char *argv[])
{
	rosbag::Bag mybag;
	mybag.open (argv[1], rosbag::bagmode::Read);
	float fps = atof (argv[3]);

	vector<string> topics;
	topics.push_back (argv[2]);
	rosbag::View bagview (mybag, rosbag::TopicQuery(topics));
	int numMsg = bagview.size();

	cv::VideoWriter vidwriter;
	int countMsg = 0;

	foreach (rosbag::MessageInstance const m, bagview) {
		sensor_msgs::CompressedImage::ConstPtr cimg = m.instantiate<sensor_msgs::CompressedImage>();

		cv::InputArray imgData (cimg->data);
		cv::Mat img = cv::imdecode (imgData, -1);

		// color, resolution?
		if (vidwriter.isOpened() == false) {
			int w = img.cols;
			int h = img.rows;
			bool isColor;
			int typ;
			if (img.type()==CV_8UC1)
				isColor = false;
			else if (img.type()==CV_8UC3)
				isColor = true;
			vidwriter.open ("./output.avi",
				CV_FOURCC(codec[0], codec[1], codec[2], codec[3]),
				fps,img.size(), isColor);
		}

		vidwriter.write (img);
		countMsg++;
		cout << countMsg << '/' << numMsg << "\r";
		if (countMsg==numMsg) cout << endl;
	}

	mybag.close();
}
