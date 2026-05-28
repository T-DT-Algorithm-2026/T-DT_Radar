#include "filter_plus.h"

float dist(const pcl::PointXY &a, const pcl::PointXY &b)
{
    return std::hypot(a.x - b.x, a.y - b.y);
}//计算距离



class car {
public:
    car() 
        : currentkf(nullptr), point{0,0},send_point{0,0}, color(-1), number(-1) {}

    // 修改 getcar 函数：裸指针转智能指针
    void getcar(Kalman_filter_plus* KFs) 
    {
        if(KFs)
        {
            currentkf = std::make_shared<Kalman_filter_plus>(*KFs);  // 将裸指针转为智能指针
        }
    }

    void set_radio_point(
        const pcl::PointXY &input,
        int input_color,
        int input_number,
        std::chrono::steady_clock::time_point input_time)
    {
        if(input.x == 0 && input.y == 0)
        {
            has_radio_point = false;
            return;
        }
        radio_point = input;
        radio_color = input_color;
        radio_number = input_number;
        radio_timer = input_time;
        has_radio_point = true;
    }


    void deal_car() 
    {
        if(radio_is_valid())
        {
            point = radio_point;
            send_point = pcl::PointXY{radio_point.x, radio_point.y};
            color = radio_color;
            number = radio_number;
        }
        else if (currentkf&&currentkf->miss_last_time < 0.1&&currentkf->get_time() < kalman_timeout)
        {
            point = currentkf->predict_point;
            // lastkf = currentkf;
            send_point = pcl::PointXY{currentkf->predict_point.x, currentkf->predict_point.y};
            color = currentkf->now_color;
            number = currentkf->now_number;
            timer = std::chrono::steady_clock::now();
        }
        else 
        {
            send_point = pcl::PointXY{0, 0};
        }
        currentkf = nullptr;
        
    }

    pcl::PointXY send_point ;  // 要发送的点
    int color;
    int number;

private:
    std::shared_ptr<Kalman_filter_plus> currentkf;  // 使用智能指针
    // std::shared_ptr<Kalman_filter_plus> lastkf;     // 使用智能指针
    pcl::PointXY point;
    std::chrono::steady_clock::time_point timer;             // 当前点
    pcl::PointXY radio_point{0, 0};
    std::chrono::steady_clock::time_point radio_timer;
    bool has_radio_point = false;
    int radio_color = -1;
    int radio_number = -1;
    float radio_timeout = 0.5;
    float kalman_timeout = 0.5;

    float get_time() 
    { //两帧之间时间
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - timer);
        // std::cout << duration.count()/1000.0 <<"ms"<< std::endl;
        float t = duration.count() / 1000.0;
        // if(t > 0.3) return 0.1;
        return t;
    }

    bool radio_is_valid()
    {
        if(!has_radio_point)
        {
            return false;
        }
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - radio_timer);
        if(duration.count() / 1000.0 > radio_timeout)
        {
            has_radio_point = false;
            return false;
        }
        return true;
    }
};
