---
title: Story 2.8 — Connect pipeline to ROS2 publishers
description: Wire the DepthAI pipeline to actually publish detection and segmentation messages
date: 2026-01-14
epic: epic-02
status: DONE
priority: P0
---

# Story 2.8 — Connect pipeline to ROS2 publishers

## Objective

Connect the DepthAI pipeline (Story 2.2) to the ROS2 node publishers (Story 1.2) so that detection and segmentation results are actually published to ROS2 topics.

## Background

Currently:
- `pipeline.py` creates the DepthAI pipeline and returns raw results ✅
- `node.py` creates ROS2 publishers but they're placeholders ✅
- **Nothing connects them** ❌

This story bridges the gap.

## Acceptance Criteria

- [ ] ROS2 node starts the DepthAI pipeline on initialization
- [ ] Detection results are decoded (Story 2.7) and published to `/vision/detections`
- [ ] Segmentation results are published to `/vision/segmentation/mask`
- [ ] 3D detections are published to `/vision/detections_3d`
- [ ] Pipeline runs in a separate thread to not block ROS2 callbacks
- [ ] Clean shutdown stops pipeline properly
- [ ] Can verify topics with `ros2 topic echo`

## Technical Design

### Node Architecture

```python
class AiCameraVisionNode(Node):
    def __init__(self):
        super().__init__("ai_camera_vision")
        
        # ... existing parameter setup ...
        
        # Create pipeline
        self._pipeline = create_pipeline(
            self._cfg.yolo_blob_path,
            self._cfg.seg_blob_path,
            enable_depth=True,
        )
        
        # Start pipeline runner in background thread
        self._runner = PipelineRunner(self._pipeline, force_usb2=False)
        self._running = True
        self._pipeline_thread = threading.Thread(target=self._run_pipeline)
        self._pipeline_thread.start()
        
        # Timer for periodic stats logging
        self.create_timer(10.0, self._log_stats)
    
    def _run_pipeline(self):
        """Background thread that reads pipeline and publishes."""
        with self._runner:
            while self._running:
                # Get detection result
                det_result = self._runner.get_detection_result(timeout_ms=100)
                if det_result:
                    detections = decode_yolo_output(det_result.raw_data)
                    self._publish_detections(detections, det_result)
                
                # Get segmentation result
                seg_result = self._runner.get_segmentation_result(timeout_ms=0)
                if seg_result:
                    self._publish_segmentation(seg_result)
                
                # Get depth for 3D detections
                depth_result = self._runner.get_depth_result(timeout_ms=0)
                if depth_result and det_result:
                    self._publish_detections_3d(detections, depth_result)
```

### Message Publishing

```python
def _publish_detections(self, detections: list, result: InferenceResult):
    """Convert detections to Detection2DArray and publish."""
    msg = Detection2DArray()
    msg.header.stamp = self._timestamp_to_ros(result.timestamp_ns)
    msg.header.frame_id = result.frame_id
    
    for det in detections:
        d = Detection2D()
        d.bbox.center.position.x = (det.x_min + det.x_max) / 2
        d.bbox.center.position.y = (det.y_min + det.y_max) / 2
        d.bbox.size_x = det.x_max - det.x_min
        d.bbox.size_y = det.y_max - det.y_min
        
        result = ObjectHypothesisWithPose()
        result.hypothesis.class_id = str(det.class_id)
        result.hypothesis.score = det.confidence
        d.results.append(result)
        
        msg.detections.append(d)
    
    self._detections_pub.publish(msg)
```

## Files to Modify

- `src/ai_camera_vision/ai_camera_vision/node.py`
  - Add pipeline initialization
  - Add background thread for pipeline
  - Add publishing logic

## Files to Reference

- `src/ai_camera_vision/ai_camera_vision/pipeline.py` (PipelineRunner)
- `src/ai_camera_vision/ai_camera_vision/detections.py` (from Story 2.7)
- `src/ai_camera_vision/ai_camera_vision/depth.py` (3D detection utilities)

## Test Plan

### Unit Tests

1. Mock pipeline, verify publishers called correctly
2. Message format validation
3. Thread shutdown behavior

### Integration Tests (Raspberry Pi)

1. Run node: `ros2 run ai_camera_vision ai_camera_vision_node`
2. Verify topics: `ros2 topic list | grep vision`
3. Echo detections: `ros2 topic echo /vision/detections`
4. Check FPS: `ros2 topic hz /vision/detections`

## Dependencies

- Story 2.2 (pipeline exists)
- Story 2.7 (YOLO decoder exists)
- Story 2.5 (depth utilities exist)
- Story 1.2 (node skeleton with publishers)

## Notes

- This is where everything comes together
- Without this, the ROS2 node is just a skeleton
- Must handle pipeline errors gracefully
- Consider adding reconnection logic for USB disconnects
