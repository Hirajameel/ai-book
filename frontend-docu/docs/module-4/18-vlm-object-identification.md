---
id: vlm-object-identification
title: Vision-Language Models (VLM)
sidebar_label: Chapter 18 VLM Object Identification
---

# Vision-Language Models (VLM)

## Overview

This chapter covers Vision-Language Models for object identification based on verbal descriptions. We'll explore using models like CLIP and Grounding DINO to find objects by name ("Find the red cup") and integrate them with the robotic system.

## Development Environment Setup

### Prerequisites

Before implementing the VLM integration, ensure your system meets the following requirements:

#### Hardware Requirements
- **NVIDIA Jetson Orin** (Edge deployment) or **RTX GPU** (Simulation)
- At least 8GB RAM (16GB recommended for VLM inference)
- CUDA-compatible GPU with 8GB+ VRAM for optimal performance

#### Software Requirements
- ROS 2 Humble Hawksbill (or newer)
- Python 3.8 or higher
- CUDA 11.8+ (for GPU acceleration)

### Installation Steps

1. **Install PyTorch with CUDA support**:
```bash
pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118
```

2. **Install CLIP dependencies**:
```bash
pip install openai-clip
```

3. **Install Grounding DINO dependencies**:
```bash
pip install groundingdino-py
# Alternative: Install from source for latest features
pip install git+https://github.com/IDEA-Research/GroundingDINO.git
```

4. **Install additional computer vision dependencies**:
```bash
pip install opencv-python
pip install supervision
pip install transformers
```

## Zero-Shot Object Detection

### CLIP-Based Object Detection

CLIP (Contrastive Language-Image Pre-training) enables zero-shot object detection by comparing image patches with text descriptions:

```python
import clip
import torch
import cv2
import numpy as np
from PIL import Image
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image as ImageMsg
from std_msgs.msg import String
from cv_bridge import CvBridge

class CLIPObjectDetector:
    def __init__(self, node: Node):
        self.node = node
        self.bridge = CvBridge()

        # Load pre-trained CLIP model
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.model, self.preprocess = clip.load("ViT-B/32", device=self.device)

        # Publishers and subscribers
        self.detection_publisher = node.create_publisher(String, 'object_detections', 10)

    def detect_objects(self, image: np.ndarray, descriptions: list) -> dict:
        """
        Detect objects in image based on text descriptions using CLIP

        Args:
            image: Input image as numpy array
            descriptions: List of object descriptions to search for

        Returns:
            Dictionary with detection results
        """
        # Convert image to PIL and preprocess
        pil_image = Image.fromarray(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
        image_input = self.preprocess(pil_image).unsqueeze(0).to(self.device)

        # Tokenize text descriptions
        text_inputs = torch.cat([clip.tokenize(f"a photo of a {desc}") for desc in descriptions]).to(self.device)

        # Get model predictions
        with torch.no_grad():
            logits_per_image, logits_per_text = self.model(image_input, text_inputs)
            probs = logits_per_image.softmax(dim=-1).cpu().numpy()[0]

        # Create results dictionary
        results = {
            "objects": [],
            "confidences": probs.tolist(),
            "best_match": descriptions[np.argmax(probs)] if len(descriptions) > 0 else None,
            "best_confidence": float(np.max(probs)) if len(probs) > 0 else 0.0
        }

        for i, desc in enumerate(descriptions):
            results["objects"].append({
                "description": desc,
                "confidence": float(probs[i])
            })

        return results

class VLMDetectionNode(Node):
    def __init__(self):
        super().__init__('vlm_detection_node')
        self.clip_detector = CLIPObjectDetector(self)
        self.bridge = CvBridge()

        # Create subscribers and publishers
        self.image_subscriber = self.create_subscription(
            ImageMsg,
            '/camera/rgb/image_raw',
            self.image_callback,
            10
        )

        self.command_subscriber = self.create_subscription(
            String,
            '/object_search_commands',
            self.command_callback,
            10
        )

        self.get_logger().info("VLM Detection Node initialized")

    def command_callback(self, msg):
        """Handle object search commands"""
        try:
            # Parse command - expecting JSON with "objects" list
            import json
            command_data = json.loads(msg.data)
            objects_to_find = command_data.get("objects", [])

            self.get_logger().info(f"Searching for objects: {objects_to_find}")

            # For now, we'll use the last received image
            # In a real implementation, you'd want to process the current image
            # or implement a more sophisticated image buffering system

        except json.JSONDecodeError:
            self.get_logger().error(f"Invalid JSON command: {msg.data}")

    def image_callback(self, msg):
        """Process incoming camera images"""
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Example: Detect common household objects
            # In practice, this would come from the command callback
            objects_to_find = ["cup", "bottle", "chair", "table", "person"]

            # Perform object detection
            results = self.clip_detector.detect_objects(cv_image, objects_to_find)

            # Publish results
            self.publish_detection_results(results)

        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

    def publish_detection_results(self, results):
        """Publish detection results to ROS topic"""
        import json
        msg = String()
        msg.data = json.dumps(results)
        self.clip_detector.detection_publisher.publish(msg)

        # Log best match
        if results.get("best_match"):
            self.get_logger().info(
                f"Best match: {results['best_match']} (confidence: {results['best_confidence']:.2f})"
            )

def main(args=None):
    rclpy.init(args=args)
    node = VLMDetectionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Grounding DINO Integration

Grounding DINO provides more precise object localization compared to CLIP:

```python
import torch
import numpy as np
from PIL import Image
import cv2
import supervision as sv
from groundingdino.util.inference import load_model, predict, annotate
import groundingdino.datasets.transforms as T

class GroundingDINOObjectDetector:
    def __init__(self, node: Node, config_path: str, weights_path: str):
        self.node = node
        self.model = load_model(config_path, weights_path)
        self.box_threshold = 0.35
        self.text_threshold = 0.25

    def detect_and_localize(self, image: np.ndarray, text_prompt: str) -> dict:
        """
        Detect and localize objects using Grounding DINO

        Args:
            image: Input image as numpy array (H, W, C)
            text_prompt: Text description of objects to find

        Returns:
            Dictionary with bounding boxes and confidences
        """
        # Convert image to PIL
        image_pil = Image.fromarray(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))

        # Transform image for model
        transform = T.Compose([
            T.RandomResize([800], max_size=1333),
            T.ToTensor(),
            T.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225])
        ])
        image_transformed, _ = transform(image_pil, None)

        # Run prediction
        boxes, logits, phrases = predict(
            model=self.model,
            image=image_transformed,
            caption=text_prompt,
            box_threshold=self.box_threshold,
            text_threshold=self.text_threshold
        )

        # Convert results to dictionary format
        results = {
            "boxes": boxes.tolist() if boxes.numel() > 0 else [],
            "logits": logits.tolist() if logits.numel() > 0 else [],
            "phrases": phrases,
            "count": len(boxes) if boxes.numel() > 0 else 0
        }

        # Convert normalized coordinates to pixel coordinates
        h, w, _ = image.shape
        for i, box in enumerate(results["boxes"]):
            # Box format: [cx, cy, w, h] in normalized coordinates
            cx, cy, w_box, h_box = box
            x1 = int((cx - w_box/2) * w)
            y1 = int((cy - h_box/2) * h)
            x2 = int((cx + w_box/2) * w)
            y2 = int((cy + h_box/2) * h)
            results["boxes"][i] = [x1, y1, x2, y2]

        return results
```

## Integration with Robotic System

### ROS 2 Message Definitions

Create custom message types for object detection results:

```bash
# In your ROS 2 package, create msg/ObjectDetection.msg:
string object_name
float32 confidence
int32 x1
int32 y1
int32 x2
int32 y2
float32[] position_3d  # Optional: 3D position if available
```

### Spatial Reasoning Implementation

```python
class SpatialReasoning:
    def __init__(self, node: Node):
        self.node = node

    def convert_2d_to_3d(self, bbox_2d: list, depth_image: np.ndarray, camera_info: dict) -> dict:
        """
        Convert 2D bounding box to 3D position using depth information

        Args:
            bbox_2d: [x1, y1, x2, y2] in pixel coordinates
            depth_image: Depth image from RGB-D camera
            camera_info: Camera intrinsic parameters

        Returns:
            3D position dictionary
        """
        x1, y1, x2, y2 = bbox_2d

        # Calculate center of bounding box
        center_x = int((x1 + x2) / 2)
        center_y = int((y1 + y2) / 2)

        # Get depth at center point (with some averaging for robustness)
        depth_roi = depth_image[center_y-5:center_y+5, center_x-5:center_x+5]
        avg_depth = np.nanmedian(depth_roi[depth_roi > 0])  # Ignore invalid depth values

        # Convert pixel coordinates to 3D using camera intrinsics
        if avg_depth > 0:
            fx = camera_info['fx']
            fy = camera_info['fy']
            cx = camera_info['cx']
            cy = camera_info['cy']

            x_3d = (center_x - cx) * avg_depth / fx
            y_3d = (center_y - cy) * avg_depth / fy
            z_3d = avg_depth

            return {
                "x": float(x_3d),
                "y": float(y_3d),
                "z": float(z_3d),
                "distance": float(avg_depth)
            }

        return {"x": 0.0, "y": 0.0, "z": 0.0, "distance": 0.0}
```

## Confidence Thresholding

Implement confidence-based filtering for reliable object identification:

```python
def filter_detections_by_confidence(self, detections: dict, threshold: float = 0.7) -> dict:
    """
    Filter detections based on confidence threshold

    Args:
        detections: Raw detection results
        threshold: Minimum confidence threshold (0.0 to 1.0)

    Returns:
        Filtered detection results
    """
    if "objects" in detections:
        # For CLIP-based results
        filtered_objects = []
        for obj in detections["objects"]:
            if obj["confidence"] >= threshold:
                filtered_objects.append(obj)

        detections["objects"] = filtered_objects
        detections["filtered_count"] = len(filtered_objects)

    elif "boxes" in detections and "logits" in detections:
        # For Grounding DINO results
        filtered_boxes = []
        filtered_logits = []
        filtered_phrases = []

        for i, logit in enumerate(detections["logits"]):
            if logit >= threshold:
                filtered_boxes.append(detections["boxes"][i])
                filtered_logits.append(logit)
                filtered_phrases.append(detections["phrases"][i])

        detections["boxes"] = filtered_boxes
        detections["logits"] = filtered_logits
        detections["phrases"] = filtered_phrases
        detections["filtered_count"] = len(filtered_boxes)

    return detections
```

## Handling Ambiguous Descriptions

Implement strategies for dealing with ambiguous object descriptions:

```python
class ObjectDisambiguation:
    def __init__(self, node: Node):
        self.node = node

    def handle_ambiguous_request(self, description: str, multiple_detections: list) -> dict:
        """
        Handle cases where multiple objects match the description

        Args:
            description: Original object description
            multiple_detections: List of potential matches

        Returns:
            Resolution strategy or request for clarification
        """
        if len(multiple_detections) == 0:
            return {
                "action": "none_found",
                "message": f"No objects matching '{description}' were found"
            }
        elif len(multiple_detections) == 1:
            return {
                "action": "accept",
                "selected_object": multiple_detections[0]
            }
        else:
            # Multiple matches found - need disambiguation
            if self.is_spatially_descriptive(description):
                # Use spatial reasoning to select the most appropriate object
                return self.select_by_spatial_context(description, multiple_detections)
            else:
                # Request clarification from user
                return {
                    "action": "request_clarification",
                    "message": f"Found multiple {description}s. Which one do you mean?",
                    "options": self.describe_options(multiple_detections)
                }

    def is_spatially_descriptive(self, description: str) -> bool:
        """Check if description contains spatial information"""
        spatial_keywords = ["left", "right", "near", "far", "front", "back", "on", "under", "next to"]
        desc_lower = description.lower()
        return any(keyword in desc_lower for keyword in spatial_keywords)
```

## Performance Considerations

### Optimization Strategies

1. **Model Quantization**: Use INT8 quantization to reduce model size and improve inference speed
2. **Batch Processing**: Process multiple frames together when possible
3. **Caching**: Cache results for similar queries to reduce redundant computation
4. **Multi-scale Processing**: Process at multiple scales to detect objects of different sizes

### Hardware Acceleration

- Use TensorRT for NVIDIA GPUs to optimize inference performance
- Consider OpenVINO for Intel hardware acceleration
- For edge deployment on Jetson, use DeepStream SDK for optimized video processing

## Integration with VLA Pipeline

The VLM component integrates with the overall VLA system by:

1. Receiving camera feeds from the perception system
2. Processing verbal object descriptions from the LLM component
3. Publishing object locations to guide navigation and manipulation actions
4. Providing feedback to the orchestrator about detection confidence and success

This enables the robot to understand and interact with its environment based on natural language descriptions, completing the Vision-Language-Action loop.