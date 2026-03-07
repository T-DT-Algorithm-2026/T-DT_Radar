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

        // if (KFs && !lastkf) 
        // {
        //     currentkf = std::make_shared<Kalman_filter_plus>(*KFs);  // 将裸指针转为智能指针
        // }//3
        // else if (lastkf.get() == KFs) 
        // {
        //     currentkf = lastkf; 

        // }//1
        // else if (lastkf.get() != KFs && lastkf&&!currentkf) 
        // {
        //     float score_new = choice(KFs, point);
        //     float score_old = choice(lastkf.get(), point);  // 获取裸指针用于计算
        //     if (score_new > score_old) 
        //     {
        //         currentkf = std::make_shared<Kalman_filter_plus>(*KFs);  // 更新智能指针
        //     }
        //     else 
        //     {
        //         currentkf = lastkf; 
        //     }
        // }//2
        // else if (currentkf&&currentkf.get() != KFs)
        // {
        //     float score_new = choice(KFs, point);
        //     float score_old = choice(currentkf.get(), point);  // 获取裸指针用于计算
        //     if (score_new > score_old) 
        //     {
        //         currentkf = std::make_shared<Kalman_filter_plus>(*KFs);  // 更新智能指针
        //     }
        // }//4
        // else
        // {
        //     currentkf = nullptr;
        // }        
    }

    void test()
    {
        // std::cout<<"color:";
        // // std::cout<<color<<std::endl;
        // std::cout<<"number";
        // std::cout<<number<<std::endl;
        // // std::cout<<"send_point";
        // // std::cout<<send_point.x<<","<<send_point.y<<std::endl;
        // std::cout<<"point";
        // std::cout<<point.x<<","<<point.y<<std::endl;  
    }

    void deal_car() 
    {
        if (currentkf&&currentkf->miss_last_time < 0.1) 
        {
            point = currentkf->predict_point;
            // lastkf = currentkf;
            send_point = pcl::PointXY{currentkf->predict_point.x, currentkf->predict_point.y};
            color = currentkf->now_color;
            number = currentkf->now_number;
            currentkf = nullptr;
            timer = std::chrono::steady_clock::now();
        }
        else if(in_zone1(point)) 
        {
            float dt=get_time();
            send_point = pcl::PointXY{point.x + dt*1, point.y};
            if(send_point.x>14.5)
            {
                send_point.x=14.5;
            }
            if(dt>5.0)
            {
                send_point = pcl::PointXY{0, 0};
            }
        }
        else if(in_zone2(point)) 
        {
            send_point = pcl::PointXY{4, 12};
        }
        else if(in_zone(point)||in_zone3(point)||in_zone4(point)) 
        {
            send_point = pcl::PointXY{point.x, point.y};
        }
        else 
        {
            send_point = pcl::PointXY{0, 0};
        }
        
    }

    pcl::PointXY send_point ;  // 要发送的点
    int color;
    int number;

private:
    std::shared_ptr<Kalman_filter_plus> currentkf;  // 使用智能指针
    // std::shared_ptr<Kalman_filter_plus> lastkf;     // 使用智能指针
    pcl::PointXY point;
    std::chrono::steady_clock::time_point timer;             // 当前点

    float choice(Kalman_filter_plus* kf, pcl::PointXY point) 
    {
        float distance = dist(kf->predict_point, point);
        return kf->catch_last_time / (kf->miss_last_time );
    }

    float get_time() 
    { //两帧之间时间
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - timer);
        // std::cout << duration.count()/1000.0 <<"ms"<< std::endl;
        float t = duration.count() / 1000.0;
        // if(t > 0.3) return 0.1;
        return t;
    }

    bool in_zone(pcl::PointXY point) 
    {
        if (point.x > 11 && point.x < 20 && point.y > 2 && point.y < 4) 
        {
            return true;
        }
        return false;
    }

    bool in_zone1(pcl::PointXY point) 
    {
        if (point.x > 11 && point.x <= 13 && point.y > 13 && point.y < 14) 
        {
            return true;
        }
        return false;
    }


    bool in_zone2(pcl::PointXY point) 
    {
        if (point.x > 8 && point.x < 9.5 && point.y > 13 && point.y < 15) 
        {
            return true;
        }
        return false;
    }

    bool in_zone3(pcl::PointXY point) 
    {
        if (point.x > 18 && point.x < 22 && point.y > 12 && point.y < 13) 
        {
            return true;
        }
        return false;
    }

    bool in_zone4(pcl::PointXY point) 
    {
        if (point.x > 21 && point.x < 24 && point.y > 14 && point.y < 15) 
        {
            return true;
        }
        return false;
    }
};