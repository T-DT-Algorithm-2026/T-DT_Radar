import argparse
import os
import shutil
import subprocess
import sys


def find_trtexec():
    env_path = os.environ.get("TRTEXEC_PATH")
    if env_path:
        executable = shutil.which(env_path)
        if executable:
            return executable
        raise FileNotFoundError(f"TRTEXEC_PATH is not executable: {env_path}")

    path_trtexec = shutil.which("trtexec")
    if path_trtexec:
        return path_trtexec

    raise FileNotFoundError(
        "trtexec not found. Set TRTEXEC_PATH or add trtexec to PATH."
    )

def run_trtexec(onnx_path, save_engine_path, min_batch, opt_batch, max_batch, input_name,shape):
    command = [find_trtexec(),
               "--onnx=" + onnx_path,
               "--saveEngine=" + save_engine_path,
               "--minShapes=" + input_name + ":" + min_batch + "x3x" +shape,
               "--optShapes=" + input_name + ":" + opt_batch + "x3x" +shape,
               "--maxShapes=" + input_name + ":" + max_batch + "x3x" +shape]
    subprocess.run(command, check=True)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Run TensorRT with specified parameters.")
    parser.add_argument("--onnx", default="", help="Path to the ONNX file.")
    parser.add_argument("--saveEngine", default="", help="Path to save the TensorRT engine file.")
    parser.add_argument("--Shape", default="224x224", help="")
    parser.add_argument("--minBatch", default="1", help="Minimum batch size for input.")
    parser.add_argument("--optBatch", default="10", help="Optimum batch size for input.")
    parser.add_argument("--maxBatch", default="20", help="Maximum batch size for input.")
    parser.add_argument("--input_name", default="input", help="Name of the input tensor.")
    args = parser.parse_args()

    try:
        run_trtexec(args.onnx, args.saveEngine, args.minBatch, args.optBatch, args.maxBatch,args.input_name,args.Shape)
    except Exception as exc:
        print(exc, file=sys.stderr)
        sys.exit(1)
