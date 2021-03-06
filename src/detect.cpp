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

/**
 * 参数矩阵， 包括相机内参，相机高度，相机与水平面夹角信息
 */
Mat_<double> param[2];

/**
    * @brief make the rect safe
    * @param [rect] a rect may be not safe
    * @param [size] the image size, the biggest rect size
    * @return it will never be false
    * @details none
    */
inline bool make_rect_safe(cv::Rect &rect, const cv::Size& size)
{
    if(rect.x >= size.width || rect.y >= size.height)rect = Rect(0,0,0,0);
    if (rect.x < 0)
        rect.x = 0;
    if (rect.x + rect.width > size.width)
        rect.width = size.width - rect.x - 1;
    if (rect.y < 0)
        rect.y = 0;
    if (rect.y + rect.height > size.height)
        rect.height = size.height - rect.y - 1 ;

    return !(rect.width <= 0 || rect.height <= 0);
}

/**
 * @brief 单目测距函数， 要求测距点必须在地面上， 参考 https://github.com/liuchangji/simple-distance-measure-by-camera
 * @param u 测距点在图像坐标系上的x坐标
 * @param v 测距点在图像坐标系上的y坐标
 * @param x 测距点在相机坐标系下的x坐标
 * @param y 测距点在相机坐标系下的y坐标
 * @param z 测距点在相机坐标系下的z坐标
 * @param camera_index 相机编号，多个相机的参数不相同
 * @return 小车与测距点间的平面直线距离
 */
static float measure_distance(double u, double v, double &x, double &y, double &z, int camera_index)
{
    u = u - param[camera_index].at<double>(5, 0);
    v = v - param[camera_index].at<double>(6, 0);

    static double angle_b, angle_c, op;
    angle_b = atan(v / param[camera_index].at<double>(4, 0))*57.325;
    angle_c = param[camera_index].at<double>(0, 0) + angle_b;

    if(angle_c == 0)
        angle_c = 0.03;
    else if(angle_c == 90 )
        angle_c = 89;

    op = param[camera_index].at<double>(1, 0) / tan(angle_c/57.325);

    z = param[camera_index].at<double>(1, 0) / sin(angle_c/57.325)*cos(angle_b/57.325);
    x = u * z / param[camera_index].at<double>(3, 0);

    y = v * z / param[camera_index].at<double>(4, 0);

    return sqrt(x * x + z * z);
}

/**
 * @brief 定时休眠函数
 * @param secs 休眠时间（秒）
 */
static void sleep_ms(unsigned int secs)
{
    struct timeval tval;

    tval.tv_sec=secs/1000;

    tval.tv_usec=(secs*1000)%1000000;

    select(0,NULL,NULL,NULL,&tval);
}

/**
 * @brief 图像预处理， 按垃圾堆放区颜色， 将图像处理为四张分别以蓝、绿、红、灰为白色的二值图像
 * @param frame 原图像
 */
void Detector::preprocess(const Mat &frame)
{
    static Mat hsv_frame;

    cvtColor(frame, hsv_frame, COLOR_BGR2HSV);

    //blue : (98, 254, 152)
    inRange(hsv_frame, Scalar(48, 204, 112), Scalar(128, 255, 192), gray[0]);

    //green : (73, 247, 90)
    inRange(hsv_frame, Scalar(33, 207, 50), Scalar(103, 255, 130), gray[1]);

    //red : (174, 254, 138)
    inRange(hsv_frame, Scalar(134, 214, 98), Scalar(214, 255, 178), gray[2]);

    //gray : (87, 32, 101)
    inRange(hsv_frame, Scalar(47,0, 64), Scalar(127, 72, 141), gray[3]);
}

void Detector::highlight_yellow_region(Mat &frame)
{
    static Mat hsv_frame;

    cvtColor(frame, hsv_frame, COLOR_BGR2HSV);

    inRange(hsv_frame, Scalar(10, 43, 46), Scalar(44, 255, 255), yellow_region);
}
/**
 * @brief Detector初始化， 初始化内容包括：
 * yolo_trt_detector : yolov4/yolov4-tiny/yolov5s 到 tensorRT的转化器初始化
 * param             : 参数矩阵初始化
 * angle_map         : 像素-角度映射矩阵初始化
 * distance_map      : 像素-距离映射矩阵初始化
 * .etc              : 动作量、状态量、滤波变量、其他变量初始化
 * @param camera 相机
 */
void Detector::initialize(int camera)
{

    //加载网络模型
    if(camera == USBCAMERA)
        net =Net(DetectionModel(usb_cfg_path, usb_weight_path));
    else
        net =Net(DetectionModel(deep_cfg_path, deep_weight_path));

    net.setPreferableBackend(DNN_BACKEND_CUDA);
    net.setPreferableTarget(DNN_TARGET_CUDA);
    out_names = net.getUnconnectedOutLayersNames();

    for (int i = 0; i < out_names.size(); i++)
    {
        printf("output layer name : %s\n", out_names[i].c_str());
    }

    //加载单目测距参数
    string filename = param_path;
    FileStorage fs(filename, FileStorage::READ);

    if (!fs.isOpened()) {
        LOGE("[ERROR] : Can Not Open File : %s",filename.c_str());
    }

    fs["USB_Param"]>>param[0];
    fs["Realsense_Param"]>>param[1];

    //加载映射表
    angle_map = imread(angle_map_path, IMREAD_GRAYSCALE);
    distance_map = imread(distance_map_path, IMREAD_GRAYSCALE);

    fs.release();

    //初始化各参数
    is_find_target = false;
    is_get_putback_position = false;
    is_get_clamp_position = false;

    distance_sum = 0;
    angle_sum = 0;
    distance_count = 0;
    angle_filter_full_flag = false;
    distance_filter_full_flag = false;
    angle_count = 0;
    distance_filter_full_flag = false;
    direction = 0;

    target_type = NOTDEFINEDTYPE;
    last_target_type = NOTDEFINEDTYPE;

    target_type_array = vector<int>(10,0);

    initialized_ground_target_type = false;
    for(auto &cur_target_type: ground_target_type)
        cur_target_type = NOTDEFINEDTYPE;
}

/**
 * @brief 垃圾检测函数
 * @param frames 由于yolo2trt库传入参数为vector<Mat>， 故也将其设计为如此吗，但在功能上与使用Mat无异
 * @param camera 相机种类， 根据不同相机测距方案不同， 状态切换也不同
 * @param mission_mode 当前状态
 */
void Detector::detect_target(Mat &frame, int camera, uint8_t mission_mode, uint8_t is_push, uint8_t  is_check_target_in_yellow)
{
    is_find_target = false;

    //网络参数输入设置
    input_blob = blobFromImage(frame, 1 / 255.F, Size(320, 320), Scalar(), true, false);//输入图像设置，input为32的整数倍，不同尺寸速度不同精度不同
    //模型参数输入
    net.setInput(input_blob);
    //模型推理
    net.forward(outs, out_names);
    //检测结果清除
    boxes.clear();
    class_ids.clear();
    confidences.clear();

    //检测结果提取
    for (auto & out : outs)
    {
        // detected objects and C is a number of classes + 4 where the first 4
        auto* data = (float*)out.data;
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

    //非极大抑制
    if(push_count >= 5)
        NMSBoxes(boxes, confidences, 0.65, 0.5, indices);
    else
        NMSBoxes(boxes, confidences, 0.5, 0.5, indices);

    //更新ground_target_type数组,以下注释代码本意为根据每次的识别情况更新一个存储每个位置垃圾种类的向量，但在实际实现上困难重重，主要原因
    //在于无法更新时无法确定自己的位置和姿态，也无法保证每次都识别图像中的所有垃圾，因此也难以更新对应位置的垃圾，可能有能实现的逻辑，但暂时未
    //想到。
    /*
    if(!initialized_ground_target_type && indices.size() != 0)
    {
        int middle_x_of_all_targets;
        int sum_x_of_all_targets = 0;

        for (int indice : indices)
            sum_x_of_all_targets += boxes[indice].x + boxes[indice].width / 2;

        middle_x_of_all_targets = sum_x_of_all_targets / indices.size();

        initialized_ground_target_type = true;
    }
    */
    if(indices.empty())
    {
        //未找到目标
        is_find_target = false;
        return;
    }
    else
    {
        //找到目标
        target_type = NOTDEFINEDTYPE;
        target_confidence = 0;

        int target_index = indices[0];

        int diff_y, diff_x;
        int diff_dis = 1 << 30;

        int last_target_type_count = 0;

        if(mission_mode == PUTBACK)
        {
            // 推取目标时，只需确定垃圾种类
            for (int indice : indices)
            {
                if(last_target_type_count == 0 && class_ids[indice] + 1 == last_target_type)
                {
                    last_target_type_count++;
                    target_index = indice;
                    continue;
                }
                target_index = indice;
                break;
            }
        }
        else
        {

            //夹取目标时， 选取距离最近的目标
            for (int indice : indices)
            {
                //是否要分拣黄色区域垃圾
                if(is_check_target_in_yellow)
                {
                    highlight_yellow_region(frame);
                    Rect close_area = Rect(boxes[indice].tl() - Point(boxes[indice].width / 2, boxes[indice].height / 2), boxes[indice].size() * 2);
                    make_rect_safe(close_area,  yellow_region.size());

                    close_area_image = yellow_region(close_area);
                    Rect cur_rect = Rect((boxes[indice].tl() - close_area.tl()), boxes[indice].size());

                    make_rect_safe(cur_rect, close_area.size());
                    Mat total_zero_image = Mat::zeros(cur_rect.size(), CV_8UC1);
                    int caculate_area = 0;

                    if(close_area.area() != boxes[indice].area())
                    {
                        total_zero_image.copyTo(close_area_image(cur_rect));
                        caculate_area = 255*(close_area.area() - boxes[indice].area());
                    }
                    else
                    {
                        caculate_area = 255*(close_area.area());
                    }
                    Scalar_<double> zero_sum = sum(close_area_image);

                    if(zero_sum[0] / (caculate_area) < 0.4)
                        continue;
                }

                diff_y = fabs(FRAME_HEIGHT - (boxes[indice].y + boxes[indice].height));

                if(diff_y < diff_dis)
                {
                    target_index = indice;
                    diff_dis = diff_y;
                }
            }
        }

        target_box = boxes[target_index];
        make_rect_safe(target_box, frame.size());

        target_type_queue.push(class_ids[target_index]);
        target_type_array[class_ids[target_index]]++;

        if(target_type_queue.size() > 10)
        {
            if(target_type_array[target_type_queue.front()] > 0)
                target_type_array[target_type_queue.front()]--;
            target_type_queue.pop();
        }

        target_type = class_ids[target_index] + 1;

        target_confidence = confidences[target_index];
    }

    //计算目标角度和距离
    if(camera == DEEPCAMERA)
    {
        //深度相机的距离并不使用这个计算出的距离，而是使用深度相机自身测量出的距离，这里主要是为了解算出x,y,z坐标计算角度
        distance = measure_distance(target_box.x + target_box.width/2, target_box.y + target_box.height, x, y, z, 1);
        //处理异常值
        if(distance == NAN)
            distance = 1000;

        angle = atan(1.0*x/z)*57.29578;
    }
    else
    {
        distance = distance_map.at<uchar>(target_box.y + target_box.height, target_box.x + target_box.width/2);

        angle = angle_map.at<uchar>(target_box.y + target_box.height, target_box.x + target_box.width/2);
    }
#ifdef DEBUG_
    LOGM("Pixel X: %d\tPixel Y : %d", target_box.x + target_box.width/2, target_box.y + target_box.height);

    LOGM("[DETECT] : DIS : %lf\t ANGLE : %lf", distance, angle);
#endif
    is_find_target = true;
}

/**
 * @brief 判断小车是否到达垃圾夹取位置，仅用于NEARING2状态
 * @return 到达夹取位置，返回True; 未到达夹取位置，返回False
 */
bool Detector::if_get_clamp_position()
{
    is_get_clamp_position = false;

    if(!is_find_target)
        return false;

    double cur_distance, cur_angle;

    Point2i target_box_bottom_middle = target_box.tl() + Point2i (target_box.width / 2, target_box.height);

    //数据出界处理
    if(target_box_bottom_middle.x >= FRAME_WIDTH)
        target_box_bottom_middle.x = FRAME_WIDTH - 1;
    else if(target_box_bottom_middle.x < 0)
        target_box_bottom_middle.x = 0;

    if(target_box_bottom_middle.y >= FRAME_HEIGHT)
        target_box_bottom_middle.y = FRAME_HEIGHT - 1;
    else if(target_box_bottom_middle.y < 0)
        target_box_bottom_middle.y = 0;

    cur_angle = angle_map.at<uchar>(target_box_bottom_middle.y, target_box_bottom_middle.x);

    //角度左右判断，当x<305.5875时说明角度向左偏转为负值；反之亦然
    if(target_box_bottom_middle.x < 305.5875)
        cur_angle = -cur_angle;

    cur_distance = distance_map.at<uchar>(target_box_bottom_middle.y, target_box_bottom_middle.x);

    //距离滑动平均滤波
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

    distance = cur_distance;

    //角度滑动平均滤波
    if(!angle_filter_full_flag)
    {
        angle_array[angle_count++] = cur_angle;
        angle_sum += cur_angle;

        angle = angle_sum/ angle_count;
    }
    else if(angle_filter_full_flag)
    {
        angle_sum -= angle_array[angle_count];

        angle_array[angle_count++] = cur_angle;

        angle_sum += cur_angle;

        angle = angle_sum / angle_length;
    }

    if(angle_count == angle_length)
        angle_filter_full_flag = true;

    angle_count = angle_count % angle_length;

#ifdef DEBUG_
    LOGM("[CLAMP] : FILL : %d\tDIS : %f\t ANGLE : %f \tSUM : %lf\tANGCOUNT : %d", angle_filter_full_flag, distance, angle, angle_sum, angle_count);
    LOGM("[CLAMP] : TARGET_TYPE : %s", target_types[target_type].c_str());
#endif

    return is_get_clamp_position;
}

/**
 * @brief 是否夹取垃圾成功
 * @return 夹取成功，返回True; 未夹取成功，返回False
 */
bool Detector::if_picked_up()
{
    is_picked_up = true;
    return is_picked_up;
}

/**
 * @brief 判断小车与目标的垃圾堆放区相对位置，指导小车在x方向进行左右平移运动，直至运动到目标垃圾堆放区正前方
 * @return 到达目标垃圾堆放区正前方，返回True; 未到达目标垃圾堆放区正前方，返回False
 */
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

/**
 * @brief 获取垃圾的类别，未防止误判垃圾类别，若两个垃圾在视野内较近或夹取垃圾前误识别则可能出现此情况， 使用一个队列保存最近若干个垃圾类别，
 * 一个向量进行计数，选择其中出现次数最大的一个作为最终垃圾类别
 * @return 垃圾类别
 */
int Detector::get_target_type()
{
    auto maxPosition = max_element(target_type_array.begin(), target_type_array.end());
    target_type = maxPosition - target_type_array.begin() + 1;
#ifdef DEBUG_
    LOGM("BATTERY : %d", target_type_array[0]);
    LOGM("SPITBALL : %d", target_type_array[1]);
    LOGM("PERICARP : %d", target_type_array[2]);
    LOGM("BOTTLE : %d", target_type_array[3]);
    LOGM("CUP : %d", target_type_array[4]);
#endif
    return target_type;
}

/**
 * @brief 清除目标类别队列
 * @return none
 */
void Detector::clear_target_array()
{
    target_type_array = vector<int>(10, 0);

    while (!target_type_queue.empty())
        target_type_queue.pop();
}
