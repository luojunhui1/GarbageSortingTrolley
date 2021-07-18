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
    //blue : (98, 254, 152)
    inRange(hsv_frame, Scalar(48, 204, 112), Scalar(128, 255, 192), gray[0]);

    //green : (73, 247, 90)
    inRange(hsv_frame, Scalar(33, 207, 50), Scalar(103, 255, 130), gray[1]);

    //red : (174, 254, 138)
    inRange(hsv_frame, Scalar(134, 214, 98), Scalar(214, 255, 178), gray[2]);

    //gray : (87, 32, 101)
    inRange(hsv_frame, Scalar(47,0, 64), Scalar(127, 72, 141), gray[3]);
}

void Detector::init_kalman_filter()
{
    kalman_filter.init(4, 2, 0);
    kalman_filter.transitionMatrix = (Mat_<float>(4, 4) << 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 0.5, 0, 0, 0, 0, 0.5);
    setIdentity(kalman_filter.measurementMatrix);
    setIdentity(kalman_filter.processNoiseCov, Scalar::all(1e-5));
    setIdentity(kalman_filter.measurementNoiseCov, Scalar::all(1e-1));
    setIdentity(kalman_filter.errorCovPost, Scalar::all(1));

    predict_count = 10;
}
void Detector::initialize()
{
    net =Net(DetectionModel(cfgPath, weightPath));
    net.setPreferableBackend(DNN_BACKEND_CUDA);
    net.setPreferableTarget(DNN_TARGET_CUDA);
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
    angle_count = 0;
    distance_filter_full_flag = false;
    direction = 0;

    target_type = NOTDEFINEDTYPE;

    last_target_mb = Point2i (FRAME_WIDTH / 2, FRAME_HEIGHT);

    kalman_state = Mat(4, 1, CV_32FC1);
    kalman_process_noise = Mat(4, 1, CV_32F);
    measurement = Mat::zeros(2, 1, CV_32F);

    init_kalman_filter();

    randn( kalman_state, Scalar::all(0), Scalar::all(0.1) );
    randn(kalman_filter.statePost, Scalar::all(0), Scalar::all(0.1));

    predict_box = Rect(0, 0, 1, 1);

}
void Detector::detect_target(const Mat &frame, int camera, uint8_t mission_mode)
{
    is_find_target = false;

    if(mission_mode > NEARING2)
       return;

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

    //---------------------------非极大抑制---------------------------
    if(mission_mode == DETECTING)
        NMSBoxes(boxes, confidences, 0.5, 0.5, indices);
    else
        NMSBoxes(boxes, confidences, 0.8, 0.5, indices);


    if(indices.empty())
    {
        if(predict_count > 5)
        {
            is_find_target = false;
            last_target_mb = Point2i (FRAME_WIDTH / 2, FRAME_HEIGHT);
            return;
        }
        else
        {
            is_find_target = true;

            prediction = kalman_filter.predict();
            predict_point = Point( (int)prediction.at<float>(0), (int)prediction.at<float>(1));

            predict_count++;

            target_box = target_box + predict_point - last_target_mb;
            last_target_mb = predict_point;
        }
    }
    else
    {
        predict_count = 0;

        target_type = NOTDEFINEDTYPE;
        target_confidence = 0;

        int target_index = indices[0];

        int diff_y, diff_x;
        int diff_dis = 1 << 30;

        for (int indice : indices)
        {
            diff_y = last_target_mb.y - (boxes[indice].y + boxes[indice].height);
            diff_x = last_target_mb.x - (boxes[indice].x + boxes[indice].width / 2);

            if(diff_x * diff_x + diff_y * diff_y < diff_dis)
            {
                target_index = indice;
                diff_dis = diff_x * diff_x + diff_y * diff_y;
            }

        }

        if(class_ids[target_index] + 1 == target_type || target_type == NOTDEFINEDTYPE)
        {
            prediction = kalman_filter.predict();
            measurement.at<float>(0)= last_target_mb.x;
            measurement.at<float>(1) = last_target_mb.y;
            kalman_filter.correct(measurement);
            LOGE("correct kalman filter");
        }
        else
        {
            init_kalman_filter();
            LOGE("%d , init kalman filter", camera);
        }

        target_box = boxes[target_index];
        last_target_mb = target_box.tl() + Point2i(target_box.width / 2, target_box.height);
        target_type = class_ids[target_index] + 1;
        target_confidence = confidences[target_index];
    }

    predict_point = Point( (int)prediction.at<float>(0), (int)prediction.at<float>(1));

    cout<<predict_point.x<<" "<<predict_point.y<<endl;
    cout<<last_target_mb.x<<" "<<last_target_mb.y<<endl;
    cout<<"==========================="<<endl;
    predict_box = target_box + predict_point - last_target_mb;

    if(camera == DEEPCAMERA)
    {
        distance = measure_distance(target_box.x + target_box.width/2, target_box.y + target_box.height, x, y, z, 1);

        angle = atan(1.0*x/z)*57.29578;
    }
    else
    {
        distance = distance_map.at<uchar>(target_box.y + target_box.height, target_box.x + target_box.width/2);

        if(distance == 255)
            distance = -5;

        angle = angle_map.at<uchar>(target_box.y + target_box.height, target_box.x + target_box.width/2);
    }

//    LOGM("Pixel X: %d\tPixel Y : %d", target_box.x + target_box.width/2, target_box.y + target_box.height);

//    LOGM("X: %f\tY: %f\tZ: %f\tDIS : %lf\t ANGLE : %lf",x, y ,z, distance, angle);

    is_find_target = true;
}

bool Detector::if_get_clamp_position()
{
    //use tracker to find the same target

    //set variable : is_get_clamp_position

    is_get_clamp_position = false;

    if(!is_find_target)
        return false;

    double cur_distance, cur_angle;

    Point2i target_box_bottom_middle = target_box.tl() + Point2i (target_box.width / 2, target_box.height);

    if(target_box_bottom_middle.x >= FRAME_WIDTH)
        target_box_bottom_middle.x = FRAME_WIDTH - 1;
    else if(target_box_bottom_middle.x < 0)
        target_box_bottom_middle.x = 0;

    if(target_box_bottom_middle.y >= FRAME_HEIGHT)
        target_box_bottom_middle.y = FRAME_HEIGHT - 1;
    else if(target_box_bottom_middle.y < 0)
        target_box_bottom_middle.y = 0;

    cur_angle = angle_map.at<uchar>(target_box_bottom_middle.y, target_box_bottom_middle.x);

    if(target_box_bottom_middle.x < 305.5875)
        cur_angle = -cur_angle;

    cur_distance = distance_map.at<uchar>(target_box_bottom_middle.y, target_box_bottom_middle.x);

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
    LOGM("[CLAMP] : TARGET_TYPE : %s", target_types[target_type].c_str());

    return is_get_clamp_position;
}

bool Detector::if_picked_up()
{
    is_picked_up = true;
    return is_picked_up;
}

bool Detector::if_get_putback_position()
{
    direction = 0;
    is_get_putback_position = false;

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
            return false;
    }

    static Mat dilate_element = getStructuringElement(MORPH_RECT, Size(5, 5));
    dilate(gray_binary, gray_binary, dilate_element);

    Mat rows_part = gray_binary.rowRange(50, 100);

    Mat rows_part_border;

    convertScaleAbs(rows_part, rows_part, 1.0/255, 0);

    Sobel(rows_part, rows_part_border, CV_16S, 1 , 0, 3, 1.0, 0, BorderTypes::BORDER_CONSTANT);

    Mat cols_sum = Mat::zeros(1, 640, CV_16S);

#pragma omp parallel for default(none) shared(FRAME_WIDTH, cols_sum, rows_part_border)
    for (int i = 0; i < FRAME_WIDTH; i++) {
        for (int j = 0; j < 50; j++) {
            cols_sum.at<short>(0, i) += rows_part_border.at<short>(j, i);
        }
    }

    Point minLoc, maxLoc;
    int average_x;
    minMaxLoc(cols_sum, nullptr, nullptr, &minLoc, &maxLoc);

    average_x = (minLoc.x + maxLoc.x) / 2;

    if(average_x < FRAME_WIDTH/2 - 50)
        direction = LEFT;
    else if(average_x > FRAME_WIDTH/2 + 50)
        direction = RIGHT;
    else
        is_get_putback_position = true;

}