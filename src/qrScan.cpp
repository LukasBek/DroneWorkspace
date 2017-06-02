#include "qrScan.h"
#include "opencv2/opencv.hpp"

#include <zbar.h>

using namespace zbar;

std::string zbarScan(Mat frame, int width, int height){
  // https://github.com/ZBar/ZBar/blob/master/examples/scan_image.cpp
  // http://zbar.sourceforge.net/api/index.html
  // http://blog.ayoungprogrammer.com/2013/07/tutorial-scanning-barcodes-qr-codes.html/ //
  std::string res;
  zbar::ImageScanner scanner;
    scanner.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);
      // obtain image data
     char file[256];
     //cin>>file;
     //Mat img = imread(file,0);
     Mat imgout;
     cvtColor(frame,imgout,CV_GRAY2RGB);
     //int width = img.cols;
     //int height = img.rows;
    uchar *raw = (uchar *)frame.data;
    // wrap image data
    zbar::Image image(width, height, "Y800", raw, width * height);
    // scan the image for barcodes
    int n = scanner.scan(image);
    // extract results
    for(Image::SymbolIterator symbol = image.symbol_begin();
      symbol != image.symbol_end();
      ++symbol) {
        std::vector<Point> vp;
        // do something useful with results
        // cout << "decoded " << symbol->get_type_name() << " symbol " << symbol->get_data() << endl;
        int n = symbol->get_location_size();
        for(int i=0;i<n;i++){
          vp.push_back(Point(symbol->get_location_x(i),symbol->get_location_y(i)));
          }
        RotatedRect r = minAreaRect(vp);
        Point2f pts[4];
        r.points(pts);
        for(int i=0;i<4;i++){
          line(imgout,pts[i],pts[(i+1)%4],Scalar(255,0,0),3);
        }
          std::stringstream strstream;
          strstream << symbol->get_data();
          strstream >> res;
        // cout<<"Angle: "<<r.angle<<endl;
   }
      imshow("Zbar",imgout);
      return res;
} // End ZbarScan
