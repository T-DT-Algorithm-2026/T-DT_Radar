#include "detect.h"

namespace tdt_radar {

Detect::~Detect()
{
    compress_running_.store(false, std::memory_order_release);
    compress_cv_.notify_all();
    if (compress_thread_.joinable())
    {
        compress_thread_.join();
    }
}

void Detect::scheduleCompress(CompressTask task)
{
    if (!compress_detect_image_ || !compressed_image_pub_ || !task.image)
    {
        return;
    }

    const auto now = std::chrono::steady_clock::now();
    if (now - last_subscriber_check_time_ >= std::chrono::milliseconds(500))
    {
        has_compressed_subscriber_ = compressed_image_pub_->get_subscription_count() > 0 || compressed_image_pub_->get_intra_process_subscription_count() > 0;
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

void Detect::compressLoop()
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

void Detect::compressAndPublish(const CompressTask& task)
{
    if (!task.image || !compressed_image_pub_)
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

        if (draw_detect_image_)
        {
            for (const auto& rectangle : task.rectangles)
            {
                cv::rectangle(image, rectangle.rect, rectangle.color, rectangle.thickness);
            }
            for (const auto& text : task.texts)
            {
                cv::putText(image, text.text, text.origin, cv::FONT_HERSHEY_SIMPLEX, text.scale, text.color, text.thickness);
            }
        }

        sensor_msgs::msg::CompressedImage compressed_msg;
        compressed_msg.header = task.image->header;
        compressed_msg.format = "jpeg";
        const std::vector<int> compression_params = {cv::IMWRITE_JPEG_QUALITY, compressed_image_quality_};
        if (cv::imencode(".jpg", image, compressed_msg.data, compression_params))
        {
            compressed_image_pub_->publish(std::move(compressed_msg));
        }
    }
    catch (const std::exception& error)
    {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Compress image failed: %s", error.what());
    }
}

}  // namespace tdt_radar
