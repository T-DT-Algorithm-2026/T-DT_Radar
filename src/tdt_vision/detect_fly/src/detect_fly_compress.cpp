#include "detect_fly.h"

namespace tdt_radar {

DetectFly::~DetectFly()
{
    compress_running_.store(false, std::memory_order_release);
    compress_cv_.notify_all();
    if (compress_thread_.joinable())
    {
        compress_thread_.join();
    }
}

void DetectFly::scheduleCompress(CompressTask task)
{
    if (!compress_image_ || !compressed_img_pub_ || !task.image)
    {
        return;
    }

    const auto now = std::chrono::steady_clock::now();
    if (now - last_subscriber_check_time_ >= std::chrono::milliseconds(500))
    {
        has_compressed_subscriber_ = compressed_img_pub_->get_subscription_count() > 0 || compressed_img_pub_->get_intra_process_subscription_count() > 0;
        last_subscriber_check_time_ = now;
    }
    if (!has_compressed_subscriber_)
    {
        return;
    }

    {
        std::lock_guard<std::mutex> lock(compress_mutex_);
        if (pending_compress_task_)
        {
            dropped_compress_frames_.fetch_add(1, std::memory_order_relaxed);
        }
        // 只保留尚未开始压缩的最新帧，避免压缩背压阻塞检测线程。
        pending_compress_task_ = std::move(task);
    }
    compress_cv_.notify_one();
}

void DetectFly::compressLoop()
{
    while (true)
    {
        CompressTask task;
        {
            std::unique_lock<std::mutex> lock(compress_mutex_);
            compress_cv_.wait(lock, [this]
            {
                return !compress_running_.load(std::memory_order_acquire) || pending_compress_task_.has_value();
            });
            if (!compress_running_.load(std::memory_order_acquire) && !pending_compress_task_)
            {
                break;
            }
            task = std::move(*pending_compress_task_);
            pending_compress_task_.reset();
        }
        compressAndPublish(task);
    }
}

void DetectFly::compressAndPublish(const CompressTask& task)
{
    if (!task.image || !compressed_img_pub_)
    {
        return;
    }

    try
    {
        cv::Mat image = cv_bridge::toCvShare(task.image, "bgr8")->image;
        if (image.empty())
        {
            return;
        }

        if (draw_compressed_image_ && task.has_target)
        {
            cv::rectangle(image, task.target_rect, cv::Scalar(0, 255, 0), 1);
            cv::circle(image, task.detect_point, 3, cv::Scalar(255, 255, 0), -1);
            cv::circle(image, task.aim_point, 3, cv::Scalar(255, 0, 255), -1);
        }

        sensor_msgs::msg::CompressedImage compressed_msg;
        compressed_msg.header = task.image->header;
        compressed_msg.format = "jpeg";
        const std::vector<int> compression_params = {cv::IMWRITE_JPEG_QUALITY, compressed_image_quality_};
        if (cv::imencode(".jpg", image, compressed_msg.data, compression_params))
        {
            compressed_img_pub_->publish(std::move(compressed_msg));
        }
    }
    catch (const std::exception& error)
    {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Compress image failed: %s", error.what());
    }
}

}  // namespace tdt_radar
