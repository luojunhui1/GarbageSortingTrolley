//
// Created by root on 2021/6/19.
//
#include "detect.h"

//struct Param{
//    double angle_a;
//    double height_camera;// milimeter
//    double f_mili;
//    double fx; // fx = f / dx
//    double fy; // fy = f  / dy
//}

Mat_<double> param[2];

static float measure_distance(double u, double v, double &x, double &y, double &z, int camera_index)
{
    u = u - param[camera_index].at<double>(5, 0);
    v = v - param[camera_index].at<double>(6, 0);

    static double angle_b, angle_c, op;
    angle_b = atan(v / param[camera_index].at<double>(4, 0))*57.325;
    angle_c = param[camera_index].at<double>(0, 0) + angle_b;
    if(angle_c == 0)
        angle_c = 0.01;
    op = param[camera_index].at<double>(1, 0) / tan(angle_c/57.325);

    z = param[camera_index].at<double>(1, 0) / sin(angle_c/57.325)*cos(angle_b/57.325);
    x = u * z / param[camera_index].at<double>(3, 0);

    y = v * z / param[camera_index].at<double>(4, 0);

    return sqrt(x * x + z * z);
}

static void sleep_ms(unsigned int secs)
{
    struct timeval tval;

    tval.tv_sec=secs/1000;

    tval.tv_usec=(secs*1000)%1000000;

    select(0,NULL,NULL,NULL,&tval);
}


void Detector::preprocess(const Mat &frame)
{
    static Mat channels[3];

    static Mat hsv_frame;

    cvtColor(frame, hsv_frame, COLOR_BGR2HSV);
    split(hsv_frame, channels);

    inRange(hsv_frame, Scalar(90, 200, 46), Scalar(124, 255, 200), gray[0]);
    inRange(hsv_frame, Scalar(35, 43, 46), Scalar(77, 255, 255), gray[1]);
    inRange(hsv_frame, Scalar(156, 43, 46), Scalar(180, 255, 255), gray[2]);
    inRange(hsv_frame, Scalar(10,0, 70), Scalar(140, 53, 140), gray[3]);
}

void Detector::initialize()
{
    net =Net(DetectionModel(cfgPath, weightPath));
    net.setPreferableBackend(DNN_BACKEND_CUDA);
    net.setPreferableTarget(DNN_TARGET_CUDA_FP16);
    out_names = net.getUnconnectedOutLayersNames();

    for (int i = 0; i < out_names.size(); i++)
    {
        printf("output layer name : %s\n", out_names[i].c_str());
    }

    string filename ="../resource/param.xml";
    FileStorage fs(filename, FileStorage::READ);

    if (!fs.isOpened()) {
        LOGE("[ERROR] : Can Not Open File : %s",filename.c_str());
    }

    fs["USB_Param"]>>param[0];
    fs["Realsense_Param"]>>param[1];

    angle_map = imread("../resource/angle_map.jpg", IMREAD_GRAYSCALE);
    distance_map = imread("../resource/distance_map.jpg", IMREAD_GRAYSCALE);

    fs.release();

    is_find_target = false;
    is_get_putback_position = false;
    is_get_clamp_position = false;

    distance_sum = 0;
    distance_count = 0;
    distance_filter_full_flag = false;

}
void Detector::detect_target(const Mat &frame, int mission_state)
{
    is_find_target = false;

    distance_count = 0;
    distance_sum = 0;
    distance_filter_full_flag = false;

    angle_sum = 0;
    angle_count = 0;
    angle_filter_full_flag = false;

    input_blob = blobFromImage(frame, 1 / 255.F, Size(320, 320), Scalar(), true, false);//输入图像设置，input为32的整数倍，不同尺寸速度不同精度不同

    net.setInput(input_blob);

    net.forward(outs, out_names);

    boxes.clear();
    class_ids.clear();
    confidences.clear();

    for (auto & out : outs)
    {
        // detected objects and C is a number of classes + 4 where the first 4
        float* data = (float*)out.data;
        for (int j = 0; j < out.rows; ++j, data += out.cols)
        {
            Mat scores = out.row(j).colRange(5, out.cols);
            minMaxLoc(scores, nullptr, &confidence, 0, &class_id_point);
            if (confidence > 0.5)
            {
                int centerX = (int)(data[0] * frame.cols);
                int centerY = (int)(data[1] * frame.rows);
                int width = (int)(data[2] * frame.cols);
                int height = (int)(data[3] * frame.rows);
                int left = centerX - width / 2;
                int top = centerY - height / 2;

                class_ids.push_back(class_id_point.x);
                confidences.push_back((float)confidence);
                boxes.push_back(Rect(left, top, width, height));
            }
        }
    }

    if(boxes.empty())
    {
        is_find_target = false;
        return;
    }

    //---------------------------非极大抑制---------------------------
    NMSBoxes(boxes, confidences, 0.5, 0.5, indices);

    target_type = NOTDEFINEDTYPE;
    target_confidence = 0;

    int target_index = indices[0];

    int target_bottom_middle_y, target_bottom_middle_x;
    int min_dis_target2bottom_middle_center = 1<<30;

    for (int indice : indices)
    {
        target_bottom_middle_y = FRAME_HEIGHT - boxes[indice].y + boxes[indice].height;
        target_bottom_middle_x = FRAME_WIDTH/2 - boxes[indice].x + boxes[indice].width/2;

        if(mission_state == NEARING)
        {
            if(fabs(target_bottom_middle_x) > FRAME_WIDTH/20)
                continue;
        }

        if(target_bottom_middle_x*target_bottom_middle_x + target_bottom_middle_y*target_bottom_middle_y < min_dis_target2bottom_middle_center)
        {
            target_index = indice;
            min_dis_target2bottom_middle_center = target_bottom_middle_x*target_bottom_middle_x + target_bottom_middle_y*target_bottom_middle_y;
        }

    }

    target_box = boxes[target_index];
    target_type = class_ids[target_index] + 1;
    target_confidence = confidences[target_index];

    distance = measure_distance(target_box.x + target_box.width/2, target_box.y + target_box.height, x, y, z, 1);

    angle = atan(1.0*x/z)*57.29578;
    LOGM("Pixel X: %d\tPixel Y : %d", target_box.x + target_box.width/2, target_box.y + target_box.height);

    LOGM("X: %f\tY: %f\tZ: %f\tDIS : %lf\t ANGLE : %lf",x, y ,z, distance, angle);

    is_find_target = true;
}

void Detector::if_get_clamp_position()
{
    //use tracker to find the same target

    //set variable : is_get_clamp_position

    if(!is_find_target)
        is_get_clamp_position = false;

    double cur_distance, cur_angle;

    Point2i target_box_bottom_middle = target_box.tl() + Point2i (target_box.width / 2, target_box.height);

    cur_angle = angle_map.at<uchar>(target_box_bottom_middle.x, target_box_bottom_middle.y);

    if(target_box_bottom_middle.x < 305.5875)
        cur_angle = -cur_angle;

    cur_distance = distance_map.at<uchar>(target_box_bottom_middle.x, target_box_bottom_middle.y);

    if(cur_distance == 255)
        cur_distance = -5;

    if(!distance_filter_full_flag)
    {
        distance_array[distance_count++] = cur_distance;
        distance_sum += cur_distance;

        distance = distance_sum/ distance_count;
    } else if(distance_filter_full_flag)
    {
        distance_sum -= distance_array[distance_count];

        distance_array[distance_count++] = cur_distance;

        distance_sum += cur_distance;

        distance = distance_sum / distance_length;
    }

    if(distance_count == distance_length)
        distance_filter_full_flag = true;

    distance_count = distance_count % distance_length;


    if(!angle_filter_full_flag)
    {
        angle_array[angle_count++] = cur_angle;
        angle_sum += cur_angle;

        angle = angle_sum/ angle_count;
    } else if(angle_filter_full_flag)
    {
        angle_sum -= angle_array[angle_count];

        angle_array[angle_count++] = cur_angle;

        angle_sum += cur_angle;

        angle = angle_sum / angle_length;
    }

    if(angle_count == angle_length)
        angle_filter_full_flag = true;

    angle_count = angle_count % angle_length;

    if(fabs(angle) < 20 && fabs(distance) < 25 && distance_count > 5)
        is_get_clamp_position = true;

    LOGM("[CLAMP] : DIS : %lf\t ANGLE : %lf", distance, angle);

}

void Detector::if_picked_up()
{
    is_picked_up = true;
}

void Detector::if_get_putback_position()
{
    direction = 0;
    is_get_putback_position = 0;

    switch (target_type) {
        case SPITBALL:
        case BOTTLE:
            gray_binary = gray[0];
            break;
        case PERICARP:
            gray_binary = gray[1];
            break;
        case BATTERY:
            gray_binary = gray[2];
            break;
        case CUP:
            gray_binary = gray[3];
            break;
        default:
            LOGE("target Type is NOT DEFINED");
            return;
    }

    vector<vector<Point>> contoursPoints;

    RotatedRect possibleArea;

    double area_angle;

    findContours(gray_binary, contoursPoints, RETR_EXTERNAL, CHAIN_APPROX_NONE);

#pragma omp parallel for
    for (auto & i : contoursPoints)
    {
        if (i.size() < 5)
            continue;

        double length = arcLength(i, true);

        if (length > 10 && length < 4000) {
            possibleArea = fitEllipse(i);

            area_angle = (possibleArea.angle > 90.0f) ? (possibleArea.angle - 180.0f) : (possibleArea.angle);

            if(fabs(area_angle) >= 20)continue;

            if(possibleArea.center.x < param[1].at<double>(5, 0) - 30)
                direction = 1;
            else if(possibleArea.center.x > param[1].at<double>(5, 0) + 30)
                direction = 2;
            else
                is_get_putback_position = 1;
        }

        return;
    }

    LOGW("Not Found Any Area to Put Down Target");
}

float Detector::get_target_distance()
{
    return distance;
}

float Detector::get_target_angle()
{
    return angle;
}