#include <iostream>
#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>

using namespace cv;
using namespace std;

int main() 
{
    Mat img = imread("C:\\Users\\haeer\\Pictures\\1.jpg", IMREAD_COLOR);
    if(img.empty())
    {
        cout << "Could not read image: " << endl;
    }
    else
    {
        auto size = img.size();
        cout << "Image size: " << size.width << " x " << size.height << endl;
    }

    Eigen::Matrix3d matrix;
    matrix << 1, 2, 3,
              4, 5, 6,
              7, 8, 9;
    cout << "Matrix: " << endl << matrix << endl;

    return 0;
} 