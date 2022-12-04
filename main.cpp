#include "main.h"

int main()
{
    Mat lane_image = imread("lane_pictures/lane_picture_9.jpg");

    vector<vector<int>> average_lines =detect_lane_in_picture(lane_image);
  
    Mat line_image = display_lines(lane_image, average_lines);

    imshow("Lines of lane image", line_image);
    waitKey(0);

    Mat combo_image;
    addWeighted(lane_image, 0.8, line_image, 1, 1, combo_image);

    imshow("Combo", combo_image);
    waitKey(0);

    //Display the lane line and the recommanded steering angle
    float steering_angle = get_steering_angle(combo_image, average_lines);
    string text_2_display = "The steering angle is " + to_string(steering_angle) + " degrees";
    putText(combo_image, text_2_display, Point(10, 20), FONT_HERSHEY_DUPLEX, 0.5, Scalar(0, 0, 255), 1);

    imshow("Final Combo", combo_image);
    waitKey(0);



    // Mat bird_eye_view = bird_eye_view_perspective(combo_image, average_lines);
    // imshow("Bird Eye", bird_eye_view);
    // waitKey(0);


    return 0;
       
}



