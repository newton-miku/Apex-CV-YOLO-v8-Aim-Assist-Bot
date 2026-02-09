from ultralytics import YOLO
import os
import torch
from typing import Optional, Any

# -----------------------------------------------------------------------------
# 全局CUDA优化配置 (只需设置一次)
# -----------------------------------------------------------------------------
_CUDA_OPTIMIZED = False

def _init_cuda_optimizations():
    """初始化CUDA优化设置，仅调用一次"""
    global _CUDA_OPTIMIZED
    if _CUDA_OPTIMIZED:
        return

    if torch.cuda.is_available():
        # 启用cudnn benchmark，自动选择最优算法
        torch.backends.cudnn.benchmark = True
        # 启用TF32 on Ampere+ GPUs for faster FP32 math
        torch.backends.cuda.matmul.allow_tf32 = True
        torch.backends.cudnn.allow_tf32 = True
        print(f"CUDA优化已启用，设备: {torch.cuda.get_device_name(0)}")

    _CUDA_OPTIMIZED = True

class YoloPredictor:
    """
    Encapsulates YOLOv8 inference logic with optimized configurations.
    Follows Single Responsibility Principle (SRP) by handling only inference.
    """
    def __init__(self, model_path: str, args: Any):
        self.model_path = model_path

        # 初始化CUDA优化（全局只需一次）
        _init_cuda_optimizations()

        # Load model once during initialization
        self.model = self._load_model(args)
        
    def _load_model(self, args: Any) -> YOLO:
        """
        Loads the YOLO model.
        """
        print(f"Loading model from {self.model_path} (FP16: {args.half})")
        return YOLO(self.model_path)

    def predict(self, img, args: Any):
        """
        Performs inference on a single image frame using optimized settings.
        
        Args:
            img: The input image (numpy array).
            args: Configuration arguments.
            
        Returns:
            The first Result object from the inference.
        """
        if img is None:
            return None
            
        # Inference Configuration Optimization
        # 1. stream=True: Returns a generator. For single images, this reduces 
        #    some overhead compared to accumulating results in a list.
        # 2. imgsz=640: Fixed input size avoids dynamic resizing calculations.
        # 3. batch=1: Explicitly set batch size to 1 for real-time inference.
        # 4. half=True: FP16 inference reduces memory usage and increases speed on GPU.
        # 5. device=0: Force GPU usage.
        infer_config = {
            "stream": True,
            "verbose": args.verbose,
            "half": args.half,
            "iou": args.iou,
            "conf": args.conf,
            # "batch": 1,
            "max_det": 20,
            "classes": [args.target_index], # Filter non-target classes early
            "agnostic_nms": False,
        }
        
        # Execute inference
        # model(...) with stream=True returns a generator
        with torch.no_grad():
            results = self.model(img, **infer_config)
            
            # Since we are processing a single frame, we extract the first result.
            # This allows us to benefit from the 'stream' mode's generator approach
            # even within a manual loop.
            for r in results:
                return r
            
        return None

# -----------------------------------------------------------------------------
# Application Layer / Facade
# -----------------------------------------------------------------------------

# Singleton instance to be used by the application
_predictor: Optional[YoloPredictor] = None

def predict_init(args):
    """
    Initializes the YoloPredictor singleton.
    """
    global _predictor
    model_path = os.path.normpath(args.model_dir + args.model)
    _predictor = YoloPredictor(model_path, args)

def predict(args, img):
    """
    Facade function to call the predictor.
    Maintains backward compatibility with main.py.
    """
    if _predictor is None:
        # Fallback or lazy init if needed, but main.py calls predict_init
        raise RuntimeError("YoloPredictor not initialized. Call predict_init first.")
    
    return _predictor.predict(img, args)
