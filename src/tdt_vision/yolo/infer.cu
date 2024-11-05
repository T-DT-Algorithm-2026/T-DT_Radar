#include <NvInfer.h>
#include <cuda_runtime.h>
#include <stdarg.h>

#include <fstream>
#include <numeric>
#include <sstream>
#include <unordered_map>

#include "infer.hpp"

#define checkRuntime(call)                                                                   \
do {                                                                                         \
    auto ___call__ret_code__ = (call);                                                       \
    if (___call__ret_code__ != cudaSuccess) {                                                \
        INFO("CUDA Runtime error💥 %s # %s, code = %s [ %d ]", #call,                        \
            cudaGetErrorString(___call__ret_code__), cudaGetErrorName(___call__ret_code__),  \
            ___call__ret_code__);                                                            \
        abort();                                                                             \
    }                                                                                        \
} while (0)

#define checkKernel(...)                  \
do {                                      \
    { (__VA_ARGS__); }                    \
    checkRuntime(cudaPeekAtLastError());  \
} while (0)

#define Assert(op)                    \
do {                                  \
    bool cond = !(!(op));             \
    if (!cond) {                      \
        INFO("Assert failed, " #op);  \
        abort();                      \
    }                                 \
} while (0)

#define Assertf(op, ...)                                \
do {                                                    \
    bool cond = !(!(op));                               \
    if (!cond) {                                        \
        INFO("Assert failed, " #op " : " __VA_ARGS__);  \
        abort();                                        \
    }                                                   \
} while (0)

namespace trt {
    using namespace std;
    using namespace nvinfer1;

    static string file_name(const string &path, bool include_suffix) {
        if (path.empty()) return "";

        int p = path.rfind('/');
        int e = path.rfind('\\');
        p = std::max(p, e);
        p += 1;

        // include suffix
        if (include_suffix) return path.substr(p);

        int u = path.rfind('.');
        if (u == -1) return path.substr(p);

        if (u <= p) u = path.size();
        return path.substr(p, u - p);
    }

    void __log_func(const char *file, int line, const char *fmt, ...) {
        va_list vl;
        va_start(vl, fmt);
        char buffer[2048];
        string filename = file_name(file, true);
        int n = snprintf(buffer, sizeof(buffer), "[%s:%d]: ", filename.c_str(), line);
        vsnprintf(buffer + n, sizeof(buffer) - n, fmt, vl);
        fprintf(stdout, "%s\n", buffer);
    }

    static std::string format_shape(const Dims &shape) {
        stringstream output;
        char buf[64];
        const char *fmts[] = {"%d", "x%d"};
        for (int i = 0; i < shape.nbDims; ++i) {
            snprintf(buf, sizeof(buf), fmts[i != 0], shape.d[i]);
            output << buf;
        }
        return output.str();
    }

    Timer::Timer() {
        checkRuntime(cudaEventCreate((cudaEvent_t *)&start_));
        checkRuntime(cudaEventCreate((cudaEvent_t *)&stop_));
    }

    Timer::~Timer() {
        checkRuntime(cudaEventDestroy((cudaEvent_t)start_));
        checkRuntime(cudaEventDestroy((cudaEvent_t)stop_));
    }

    void Timer::start(void *stream) {
        stream_ = stream;
        checkRuntime(cudaEventRecord((cudaEvent_t)start_, (cudaStream_t)stream_));
    }

    float Timer::stop(const char *prefix, bool print) {
        checkRuntime(cudaEventRecord((cudaEvent_t)stop_, (cudaStream_t)stream_));
        checkRuntime(cudaEventSynchronize((cudaEvent_t)stop_));

        float latency = 0;
        checkRuntime(cudaEventElapsedTime(&latency, (cudaEvent_t)start_, (cudaEvent_t)stop_));

        if (print) {
            printf("[%s]: %.5f ms\n", prefix, latency);
        }
        return latency;
    }

    BaseMemory::BaseMemory(void *cpu, size_t cpu_bytes, void *gpu, size_t gpu_bytes) {
        reference(cpu, cpu_bytes, gpu, gpu_bytes);
    }

    void BaseMemory::reference(void *cpu, size_t cpu_bytes, void *gpu, size_t gpu_bytes) {
        release();

        if (cpu == nullptr || cpu_bytes == 0) {
            cpu = nullptr;
            cpu_bytes = 0;
        }

        if (gpu == nullptr || gpu_bytes == 0) {
            gpu = nullptr;
            gpu_bytes = 0;
        }

        this->cpu_ = cpu;
        this->cpu_capacity_ = cpu_bytes;
        this->cpu_bytes_ = cpu_bytes;
        this->gpu_ = gpu;
        this->gpu_capacity_ = gpu_bytes;
        this->gpu_bytes_ = gpu_bytes;

        this->owner_cpu_ = !(cpu && cpu_bytes > 0);
        this->owner_gpu_ = !(gpu && gpu_bytes > 0);
    }

    BaseMemory::~BaseMemory() { release(); }

    void *BaseMemory::gpu_realloc(size_t bytes) {
        if (gpu_capacity_ < bytes) {
            release_gpu();

            gpu_capacity_ = bytes;
            checkRuntime(cudaMalloc(&gpu_, bytes));
        }
        gpu_bytes_ = bytes;
        return gpu_;
    }

    void *BaseMemory::cpu_realloc(size_t bytes) {
        if (cpu_capacity_ < bytes) {
            release_cpu();

            cpu_capacity_ = bytes;
            checkRuntime(cudaMallocHost(&cpu_, bytes));
            Assert(cpu_ != nullptr);
        }
        cpu_bytes_ = bytes;
        return cpu_;
    }

    void *BaseMemory::unified_realloc(size_t bytes) {
        if (unified_capacity_ < bytes) {
            release_unified();

            unified_capacity_ = bytes;
            checkRuntime(cudaMallocManaged(&unified_, bytes));
            Assert(unified_ != nullptr);
        }
        unified_bytes_ = bytes;
        return unified_;
    }

    void BaseMemory::release_cpu() {
        if (cpu_) {
            if (owner_cpu_) {
                checkRuntime(cudaFreeHost(cpu_));
            }
            cpu_ = nullptr;
        }
        cpu_capacity_ = 0;
        cpu_bytes_ = 0;
    }

    void BaseMemory::release_gpu() {
        if (gpu_) {
            if (owner_gpu_) {
                checkRuntime(cudaFree(gpu_));
            }
            gpu_ = nullptr;
        }
        gpu_capacity_ = 0;
        gpu_bytes_ = 0;
    }

    void BaseMemory::release_unified() {
        if (unified_) {
            if (owner_unified_) {
                checkRuntime(cudaFree(unified_));
            }
            unified_ = nullptr;
        }
        unified_capacity_ = 0;
        unified_bytes_ = 0;
    }

    void BaseMemory::release() {
        release_cpu();
        release_gpu();
    }

    class __native_nvinfer_logger : public ILogger {
    public:
        virtual void log(Severity severity, const char *msg) noexcept override {
            if (severity == Severity::kINTERNAL_ERROR) {
                INFO("NVInfer INTERNAL_ERROR: %s", msg);
                abort();
            } else if (severity == Severity::kERROR) {
                INFO("NVInfer: %s", msg);
            }
        }
    };

    static __native_nvinfer_logger gLogger;

    template<typename _T>
    static void destroy_nvidia_pointer(_T *ptr) {
        if (ptr) ptr->destroy();
    }

    static std::vector<uint8_t> load_file(const string &file) {
        ifstream in(file, ios::in | ios::binary);
        if (!in.is_open()) return {};

        in.seekg(0, ios::end);
        size_t length = in.tellg();

        std::vector<uint8_t> data;
        if (length > 0) {
            in.seekg(0, ios::beg);
            data.resize(length);

            in.read((char *) &data[0], length);
        }
        in.close();
        return data;
    }

    class __native_engine_context {
    public:
        virtual ~__native_engine_context() { destroy(); }

        bool construct(const void *pdata, size_t size) {
            destroy();

            if (pdata == nullptr || size == 0) return false;

            runtime_ = shared_ptr<IRuntime>(createInferRuntime(gLogger));
            if (runtime_ == nullptr) return false;

            engine_ = shared_ptr<ICudaEngine>(runtime_->deserializeCudaEngine(pdata, size));
            if (engine_ == nullptr) return false;

            context_ = shared_ptr<IExecutionContext>(engine_->createExecutionContext());
            return context_ != nullptr;
        }

    private:
        void destroy() {
            context_.reset();
            engine_.reset();
            runtime_.reset();
        }

    public:
        shared_ptr<IExecutionContext> context_;
        shared_ptr<ICudaEngine> engine_;
        shared_ptr<IRuntime> runtime_ = nullptr;
    };

    class InferImpl : public Infer {
    public:
        shared_ptr<__native_engine_context> context_;
        unordered_map<string, int> binding_name_to_index_;

        virtual ~InferImpl() = default;

        bool construct(const void *data, size_t size) {
            context_ = make_shared<__native_engine_context>();
            if (!context_->construct(data, size)) {
                return false;
            }

            setup();
            return true;
        }

        bool load(const string &file) {
            auto data = load_file(file);
            if (data.empty()) {
                INFO("An empty file has been loaded. Please confirm your file path: %s", file.c_str());
                return false;
            }
            return this->construct(data.data(), data.size());
        }

        void setup() {
            auto engine = this->context_->engine_;
            int nbBindings = engine->getNbIOTensors();

            binding_name_to_index_.clear();
            for (int i = 0; i < nbBindings; ++i) {
                const char *tensorName = engine->getIOTensorName(i);
                binding_name_to_index_[tensorName] = i;
            }
        }


        virtual int index(const std::string &name) override {
            auto iter = binding_name_to_index_.find(name);
            Assertf(iter != binding_name_to_index_.end(), "Can not found the binding name: %s",
                    name.c_str());
            return iter->second;
        }

        virtual bool forward(const std::vector<void *> &bindings, void *stream, void *input_consum_event) override {
            auto engine = this->context_->engine_;
            auto context = this->context_->context_;

            int nbBindings = engine->getNbIOTensors();
            if (bindings.size() != nbBindings) {
                return false;
            }

            // 设置每个张量的内存地址
            for (int i = 0; i < nbBindings; ++i) {
                const char *tensorName = engine->getIOTensorName(i);
                if (!context->setTensorAddress(tensorName, bindings[i])) {
                    // 处理错误：设置张量地址失败
                    return false;
                }
            }

            // 设置输入已消耗事件（如果需要）
            if (input_consum_event) {
                context->setInputConsumedEvent(static_cast<cudaEvent_t>(input_consum_event));
            }

            // 执行推理
            return context->enqueueV3(static_cast<cudaStream_t>(stream));
        }


        virtual std::vector<int> run_dims(const std::string &name) override {
            return run_dims(index(name));
        }

        virtual std::vector<int> run_dims(int ibinding) override {
            // 获取张量名称
            std::string tensorName = this->context_->engine_->getIOTensorName(ibinding);

            // 使用 c_str() 将 std::string 转换为 const char*
            auto dim = this->context_->engine_->getTensorShape(tensorName.c_str());

            // 将维度信息转换为 std::vector<int>
            return std::vector<int>(dim.d, dim.d + dim.nbDims);
        }


        virtual std::vector<int> static_dims(const std::string &name) override {
            return static_dims(index(name));
        }

        virtual std::vector<int> static_dims(int ibinding) override {
            const char *tensorName = this->context_->engine_->getIOTensorName(ibinding);
            auto dim = this->context_->engine_->getTensorShape(tensorName);
            return std::vector<int>(dim.d, dim.d + dim.nbDims);
        }


        virtual int num_bindings() override { return this->context_->engine_->getNbIOTensors(); }


        virtual bool is_input(int ibinding) override {
            const char *tensorName = this->context_->engine_->getIOTensorName(ibinding);
            nvinfer1::TensorIOMode ioMode = this->context_->engine_->getTensorIOMode(tensorName);
            return ioMode == nvinfer1::TensorIOMode::kINPUT;
        }

        virtual bool set_run_dims(const std::string &name, const std::vector<int> &dims) override {
            return this->set_run_dims(index(name), dims);
        }

        virtual bool set_run_dims(int ibinding, const std::vector<int> &dims) override {
            const char *tensorName = this->context_->engine_->getIOTensorName(ibinding);
            nvinfer1::Dims d;


            d.nbDims = dims.size();

            for (int i = 0; i < d.nbDims; ++i) {
                d.d[i] = dims[i];
            }

            // 使用 setInputShape 设置维度
            return this->context_->context_->setInputShape(tensorName, d);
        }


        virtual int numel(const std::string &name) override { return numel(index(name)); }

        virtual int numel(int ibinding) override {
            // 获取张量名称
            const char *tensorName = this->context_->engine_->getIOTensorName(ibinding);

            // 使用 getTensorShape() 获取维度信息
            auto dim = this->context_->context_->getTensorShape(tensorName);

            // 计算元素数量
            return std::accumulate(dim.d, dim.d + dim.nbDims, 1, std::multiplies<int>());
        }


        virtual DType dtype(const std::string &name) override { return dtype(index(name)); }

        virtual DType dtype(int ibinding) override {
            // 获取张量名称
            const char *tensorName = this->context_->engine_->getIOTensorName(ibinding);

            // 获取张量的数据类型
            nvinfer1::DataType dataType = this->context_->engine_->getTensorDataType(tensorName);

            // 将 nvinfer1::DataType 转换为自定义的 DType
            return static_cast<DType>(dataType);
        }


        virtual bool has_dynamic_dim() override {
            // 获取输入和输出张量的总数
            int numIOTensors = this->context_->engine_->getNbIOTensors();

            // 遍历每个张量
            for (int i = 0; i < numIOTensors; ++i) {
                // 获取张量名称
                const char *tensorName = this->context_->engine_->getIOTensorName(i);

                // 获取张量的维度
                nvinfer1::Dims dims = this->context_->engine_->getTensorShape(tensorName);

                // 检查维度中是否存在 -1
                for (int j = 0; j < dims.nbDims; ++j) {
                    if (dims.d[j] == -1) {
                        return true;
                    }
                }
            }
            return false;
        }


        virtual void print() override {
            INFO("Infer %p [%s]", this, has_dynamic_dim() ? "DynamicShape" : "StaticShape");

            int num_input = 0;
            int num_output = 0;
            auto engine = this->context_->engine_;

            int numIOTensors = engine->getNbIOTensors();
            for (int i = 0; i < numIOTensors; ++i) {
                const char *tensorName = engine->getIOTensorName(i);
                nvinfer1::TensorIOMode ioMode = engine->getTensorIOMode(tensorName);
                if (ioMode == nvinfer1::TensorIOMode::kINPUT) {
                    num_input++;
                } else {
                    num_output++;
                }
            }

            INFO("Inputs: %d", num_input);
            for (int i = 0; i < num_input; ++i) {
                const char *name = engine->getIOTensorName(i);
                auto dim = engine->getTensorShape(name);
                INFO("\t%d.%s : shape {%s}", i, name, format_shape(dim).c_str());
            }

            INFO("Outputs: %d", num_output);
            for (int i = 0; i < num_output; ++i) {
                const char *name = engine->getIOTensorName(i + num_input);
                auto dim = engine->getTensorShape(name);
                INFO("\t%d.%s : shape {%s}", i, name, format_shape(dim).c_str());
            }
        }
    };

    Infer *loadraw(const std::string &file) {
        InferImpl *impl = new InferImpl();
        if (!impl->load(file)) {
            delete impl;
            impl = nullptr;
        }
        return impl;
    }

    std::shared_ptr<Infer> load(const std::string &file) {
        return std::shared_ptr<InferImpl>((InferImpl *) loadraw(file));
    }

    std::string format_shape(const std::vector<int> &shape) {
        stringstream output;
        char buf[64];
        const char *fmts[] = {"%d", "x%d"};
        for (int i = 0; i < (int) shape.size(); ++i) {
            snprintf(buf, sizeof(buf), fmts[i != 0], shape[i]);
            output << buf;
        }
        return output.str();
    }
}; // namespace trt
