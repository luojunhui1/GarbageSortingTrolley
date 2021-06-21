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

Mat_<double> param;

static float measure_distance(int u, int v, int &x, int &y, int &z)
{
    u = u - FRAME_WIDTH / 2;
    v = v - FRAME_HEIGHT / 2;

    static double angle_b, angle_c, op;
    angle_b = atan(v / param.at<double>(4, 0));
    angle_c = param.at<double>(0, 0) + angle_b;
    op = param.at<double>(1, 0) / sin(angle_c);

    z = op * cos(angle_b);
    x = v * z / param.at<double>(3, 0);
    y = u * z / param.at<double>(4, 0);

    return sqrt(x * x + y * y + z * z);
}

void Detector::preprocess(const Mat &frame)
{
    static Mat blue_diff_green;
    static Mat green_diff_red;
    static Mat gray[4];

    cvtColor(frame, binary, COLOR_BGR2GRAY);
    threshold(binary, binary, 150, 255, THRESH_BINARY);

//    imshow("dst",dst);

    //read, blue, green, gray
    split(frame, channels);

    subtract(channels[0], channels[1], blue_diff_green);
    threshold(blue_diff_green, gray[0], GRAY_THRESH, 255, THRESH_BINARY_INV);
    subtract(channels[1], channels[0], blue_diff_green);
    threshold(blue_diff_green, gray[1], GRAY_THRESH, 255, THRESH_BINARY_INV);

    subtract(channels[1], channels[2], blue_diff_green);
    threshold(blue_diff_green, gray[2], GRAY_THRESH, 255, THRESH_BINARY_INV);
    subtract(channels[2], channels[1], blue_diff_green);
    threshold(blue_diff_green, gray[3], GRAY_THRESH, 255, THRESH_BINARY_INV);

    bitwise_and(gray[0], gray[1], gray[0]);
    bitwise_and(gray[2], gray[3], gray[2]);

    bitwise_and(gray[0], gray[2], gray_binary);

//    imshow("gray[0]", gray[0]);
//    imshow("gray[1]", gray[1]);
//    imshow("gray[2]", gray[2]);
//    imshow("gray[3]", gray[3]);
//    imshow("gray_binary", gray_binary);
//
//    imshow("channels[0]", channels[0]);
//    imshow("channels[1]", channels[1]);
//    imshow("channels[2]", channels[2]);
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

    fs["Param"]>>param;

    fs.release();

    is_find_target = false;
}
void Detector::detect_target(const Mat &frame)
{
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

        if(target_bottom_middle_x*target_bottom_middle_x + target_bottom_middle_y*target_bottom_middle_y < min_dis_target2bottom_middle_center)
        {
            target_index = indice;
            min_dis_target2bottom_middle_center = target_bottom_middle_x*target_bottom_middle_x + target_bottom_middle_y*target_bottom_middle_y;
        }
    }

    target_box = boxes[target_index];
    target_type = class_ids[target_index];
    target_confidence = confidences[target_index];

    distance = measure_distance(target_box.x + target_box.width/2, target_box.y + target_box.height, x, y, z);

    angle = atan(x/z)*57.29578;

    is_find_target = true;
}

void Detector::if_get_clamp_position()
{
    //use tracker to find the same target

    //set variable : is_get_clamp_position
}

void Detector::if_picked_up()
{

}

void Detector::if_get_putback_position()
{

}

float Detector::get_target_distance()
{
    return distance;
}

float Detector::get_target_angle()
{
    return angle;
}