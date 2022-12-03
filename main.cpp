#include "main.h"

int main()
{
    Mat lane_image = imread("lane_pictures/lane_picture_6.jpg");

    vector<vector<int>> average_lines =detect_lane_in_picture(lane_image);
  
    Mat line_image = display_lines(lane_image, average_lines);

    imshow("Lines of lane image", line_image);
    waitKey(0);

    Mat combo_image;
    addWeighted(lane_image, 0.8, line_image, 1, 1, combo_image);

    imshow("Lines", combo_image);
    waitKey(0);

    return 0;
       
}



