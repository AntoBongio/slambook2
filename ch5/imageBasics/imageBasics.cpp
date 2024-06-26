#include <iostream>
#include <chrono>

using namespace std;

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

int main(int argc, char **argv) {
  // Image path passed as argument
  cv::Mat image;
  image = cv::imread(argv[1]); 

  if (image.data == nullptr) { 
    cerr << "Image " << argv[1] << " do not exist." << endl;
    return 0;
  }

  // See image metadata
  cout << "Image cols [" << image.cols << "], rows [" << image.rows << "], channels [" << image.channels() << "]" << endl;
  cv::imshow("image", image);    
  cv::waitKey(0);            

  // We need grayscale or rgb images
  if (image.type() != CV_8UC1 && image.type() != CV_8UC3) {
    cout << "Image type wrong." << endl;
    return 0;
  }


  chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
  for (size_t y = 0; y < image.rows; y++) {
    // use cv::Mat::ptr to get a ptr to each row
    unsigned char *row_ptr = image.ptr<unsigned char>(y);  // row_ptr
    for (size_t x = 0; x < image.cols; x++) {
      // read the pixel on (x,y) x=column, y=row
      unsigned char *data_ptr = &row_ptr[x * image.channels()]; // data_ptr 
      // visit the pixel in each channel
      for (int c = 0; c != image.channels(); c++) {
        unsigned char data = data_ptr[c]; 
      }
    }
  }
  chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
  chrono::duration<double> time_used = chrono::duration_cast < chrono::duration < double >> (t2 - t1);
  cout << "time used: " << time_used.count() << " seconds." << endl;

  // copying cv::Mat
  // operator will not copy the image data, but the reference
  cv::Mat image_another = image;
  // changing image_another will change also image
  image_another(cv::Rect(0, 0, 100, 100)).setTo(0);
  cv::imshow("image", image);
  cv::waitKey(0);

  // to actually clone the data
  cv::Mat image_clone = image.clone();
  image_clone(cv::Rect(0, 0, 100, 100)).setTo(255);
  cv::imshow("image", image);
  cv::imshow("image_clone", image_clone);
  cv::waitKey(0);

  cv::destroyAllWindows();
  return 0;
}
