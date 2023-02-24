#ifndef MAIN_H 
#define MAIN_H

#include <iostream>
#include <vector>
#include <math.h>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

Mat get_thresholded(Mat image);
Mat get_canny( Mat image);
Mat get_region_of_intereset(Mat image);
vector<vector<int>> average_slope_intercept(Mat image, vector<Vec4i> lines);

vector<double> get_average(vector<vector<double>> points);
vector<int> get_coordinates(Mat image, vector<double> average);

vector<vector<int>> detect_lane_in_picture(Mat lane_image);
vector<vector<int>> detect_lane_in_video(Mat frame);
Mat display_lines(Mat image, vector<vector<int>>lines);
Mat bird_eye_view_perspective(Mat image, vector<vector<int>> points);
float get_steering_angle(Mat image, vector<vector<int>> points);

void print_1D_vector_i_elements(vector<double> line);
void print_1D_vector_d_elements(vector<double> line);
void print_2D_vector_i_elements(vector<vector<int>> lines);
void print_2D_vector_d_elements(vector<vector<double>> lines);

float camera_offset_percent = -0.2;

vector<vector<int>> detect_lane_in_picture(Mat lane_image){
    //Detect the lines of the lane in a given picture

     if(lane_image.empty()){
        cout << "Image not available" << endl;
        return {};
    }

    imshow("Original", lane_image);
    waitKey(0);

    Mat thresholded = get_thresholded(lane_image);
    imshow("Thresholded", thresholded);
    waitKey(0);

    Mat edge = get_canny(thresholded);
    imshow("Edged", edge);
    waitKey(0);

    Mat ROI = get_region_of_intereset(edge);
    imshow("ROI", ROI);
    waitKey(0);

    vector<Vec4i> lines;
    HoughLinesP(ROI, lines, 2, CV_PI/180, 100, 30, 5);

    vector<vector<int>> average_lines;
    average_lines = average_slope_intercept(lane_image, lines);

    return average_lines;

}

Mat get_thresholded(Mat image){
    //Detect the lane lines in the image
    Mat thresholed;
    cvtColor(image, thresholed, COLOR_RGB2HSV);
    inRange(thresholed, cv::Scalar(0, 0, 0), cv::Scalar(180, 255, 40), thresholed);
    return thresholed;
}

Mat get_canny(Mat image){
    //Find the edges of the lines

    Mat og_image = image, processed_image;
    //Smoothing
    GaussianBlur(og_image, processed_image, Size(5,5), 0);
    //Edge detection avec Canny
    Canny(og_image, processed_image, 50, 150);

    return processed_image;
}

Mat get_region_of_intereset(Mat image){
    //Select the bottom part of the image

    Mat mask(image.size().height, image.size().width, CV_8UC1, Scalar(0));

    vector<Point> vertices = {  Point(0, image.size().height),
                                Point(0, int(image.size().height/2)),
                                Point(image.size().width, int(image.size().height/2)),
                                Point(image.size().width, image.size().height)}; 

    fillPoly(mask, vertices, Scalar(255, 255, 255));
    bitwise_and(image, mask, image);
    return image;

}

vector<vector<int>> average_slope_intercept(Mat image, vector<Vec4i> lines){
    //Classify and return all the detected lines endpoint coordinates by the HOUGH_TRANSFORM into two categories
    //Either left or right

    vector<vector<int>> coordinates = {};

    //If the HOUGH_TRANFROM has detected some lines
    if(!lines.empty()){

        bool addedNeg = false, addedPos = false;
        int negCounter = 0, posCounter = 0;

        vector <vector<double>> slopePositiveLines = {};
        vector <vector<double>> slopeNegativeLines = {};

        vector<double> left_lines_params = {}, right_lines_params ={};
        vector<int> left_lines_coordinates = {}, right_lines_coordinates = {};
         
        // Iterate through them and classify them either into left or right
        for(size_t i = 0; i < lines.size(); i ++){
            float x1 = lines[i][0];
            float y1 = lines[i][1];
            float x2 = lines[i][2];
            float y2 = lines[i][3];
            float slope = (y2-y1)/(x2-x1);
            float intercept = y1 - slope*x1;

            //left lines
            if( slope < 0){

                slopeNegativeLines.resize(negCounter + 1);

                slopeNegativeLines[negCounter].resize(2);

                slopeNegativeLines[negCounter][0] = slope;
                slopeNegativeLines[negCounter][1] = intercept;

                addedNeg = true;

                negCounter++;
            } 

            //right lines
            else if(slope > 0){

                slopePositiveLines.resize(posCounter + 1);

                slopePositiveLines[posCounter].resize(2);

                slopePositiveLines[posCounter][0] = slope;
                slopePositiveLines[posCounter][1] = intercept;

                addedPos = true;
                posCounter++;
            }  
        }

        if( addedPos == false && addedNeg == false ){
            cout << "No desired lines found!!"<<endl;
            return {};
        }

        else if(addedNeg && addedPos){
            left_lines_params = get_average(slopeNegativeLines);
            right_lines_params = get_average(slopePositiveLines);

            left_lines_coordinates = get_coordinates(image, left_lines_params);
            right_lines_coordinates = get_coordinates(image, right_lines_params);

            coordinates.push_back(left_lines_coordinates);
            coordinates.push_back(right_lines_coordinates);
        }
        else if(addedPos == false && addedNeg == true){
            left_lines_params = get_average(slopeNegativeLines);
    
            left_lines_coordinates = get_coordinates(image, left_lines_params);
        
            coordinates.push_back(left_lines_coordinates);
        }
        else if(addedPos == true && addedNeg == false ){
            right_lines_params = get_average(slopePositiveLines);

            right_lines_coordinates = get_coordinates(image, right_lines_params);

            coordinates.push_back(right_lines_coordinates);
        }
    }

    return coordinates;
}

vector<double> get_average(vector<vector<double>> lines){
    //Take all the lines detected for a particular orientation(left or right)
    //Each line being represented by his slope and intercept
    //And return the average value of those, thus combining all these lines into one

    float average_slope = 0, average_intercept = 0;
    int lines_size = lines.size();
    
    vector<double> lines_params = {};

    for(size_t i = 0; i < lines.size(); i++){
        average_slope += lines[i][0]/lines_size;
        average_intercept += lines[i][1]/lines_size;
    }

    lines_params.push_back(average_slope);
    lines_params.push_back(average_intercept);

    return lines_params;
}

vector<int> get_coordinates(Mat image, vector<double> average_params){
    //Calculate and return the coordinates of two points
    //Based on the slope and intercept parameters of a particular detected(averaged) line
    //needed to draw a line

    vector<int> coordinates = {};

    double slope = average_params[0], intercept = average_params[1];

    int y1 = image.size().height;
    int y2 = (y1*3)/5;
    int x1 = int((y1 - intercept)/slope);
    int  x2 = int((y2 - intercept)/slope);
     
    coordinates.push_back(x1);
    coordinates.push_back(y1);
    coordinates.push_back(x2);
    coordinates.push_back(y2);

    return coordinates;
}

Mat display_lines(Mat image, vector<vector<int>> lines){
    //Return an image where we have drawn any detected line of the lane

    int image_height = image.size().height;
    int image_width  = image.size().width;

    Mat line_image(image_height, image_width, CV_8UC3, Scalar(0));

    if(!lines.empty()){
        for(size_t i = 0; i < lines.size(); i++){
            //draw lanes
            if(lines[i].size() == 4){
                line(line_image, Point(lines[i][0], lines[i][1]), Point(lines[i][2], lines[i][3]), Scalar(255, 0, 0), 10);
            }
        }

        if(lines.size() == 2){
             //draw heading line
            int x1 = image_width/2*(1 + camera_offset_percent);
            int y1 = image_height;
            int x2 = x1;
            int y2 = image_height*3/5;
            line(line_image, Point(x1, y1), Point(x2, y2), Scalar(0, 255, 0), 10);
        }
    }
    return line_image;
}

Mat bird_eye_view_perspective(Mat image, vector<vector<int>> points){
    //Get the bird view perspective of the lane

    if(!points.empty() && points.size() == 2){

        Mat bird_view_image;

        int image_height = image.size().height;
        int image_width  = image.size().width;

        //4 points from the original image, here we've chosen the endpoints of the lane lines
        vector<Point2f> pts1 = {Point2f(points[0][0], points[0][1]),
                              Point2f(points[0][2], points[0][3]),
                              Point2f(points[1][2], points[1][3]),
                              Point2f(points[1][0], points[1][1])};
        
        //Another 4 points for the new transformed image, those are the vertices of the new image
        vector<Point2f> pts2 = {Point2f(0, image_height),
                              Point2f(0, 0),
                              Point2f(image_width, 0),
                              Point2f(image_width, image_height)};
        
        //Perspective Transform
        Mat transformation_matrix = getPerspectiveTransform(pts1, pts2);
        warpPerspective(image, bird_view_image, transformation_matrix, Size(image_width, image_height));
        
        return bird_view_image;
    }
    else{
        cout << "Not enough points" << endl;
        return {};
    }

}

float get_steering_angle(Mat image, vector<vector<int>> points){
    //Return the angle the vehicle needs to turn to stay in 

    if(!points.empty() && points.size() == 2){
        int image_height = image.size().height;
        int image_width  = image.size().width;

        int lane_midpoint = (points[0][2] + points[1][2])/2;
        int x_heading_line = image_width/2*(1 + camera_offset_percent);
        int x_offset =  x_heading_line - lane_midpoint;
        int y_offset = image_height*3/5;

        cout << x_offset << endl;
        cout << y_offset << endl;

        float steering_angle = (float) x_offset/y_offset;
        cout << steering_angle <<endl;

        steering_angle = atanf(steering_angle);
        cout << steering_angle <<endl;

        //steering_angle = roundf(steering_angle);
        //cout << steering_angle <<endl;

        float steering_angle_in_degree = steering_angle*180/CV_PI;

        return steering_angle_in_degree;
    }
    else if(!points.empty() &&points.size() == 1){
        cout << "pass" << endl;
        return 0;
    }

    cout << "Error Error Error !!!" << endl;
    return -1;
}




void print_1D_vector_i_elements(vector<int> line){
    //Helper function to print the elements of 1D vector
    //where the elements are integer type

      cout << endl;
      cout << "Printing elements..." << endl;
        for( auto params: line){
                cout << params << " ";
        }
        cout << endl;
        cout << "Done" << endl;
        cout << endl;
}

void print_1D_vector_d_elements(vector<double> line){
    //Helper function to print the elements of 1D vector
    //where the elements are double type

      cout << endl;
      cout << "Printing elements..." << endl;
        for( auto params: line){
                cout << params << " ";
        }
        cout << endl;
        cout << "Done" << endl;
        cout << endl;
}


void print_2D_vector_i_elements(vector<vector<int>> lines){
    //Helper function to print the elements of a 2D vector
    //where the elements are integer type

      cout << endl;
      cout << "Printing elements..." << endl;
        for( auto line: lines){
            for(auto elements: line){
                cout << elements << " ";
            }
            cout << endl;
        }
        cout << "Done" << endl;
        cout << endl;

}

void print_2D_vector_d_elements(vector<vector<double>> lines){
    //Helper function to print the elements of a 2D vector
    //where the elements are double type

      cout << endl;
      cout << "Printing elements..." << endl;
        for( auto line: lines){
            for(auto elements: line){
                cout << elements << " ";
            }
            cout << endl;
        }
        cout << "Done" << endl;
        cout << endl;

}

#endif 