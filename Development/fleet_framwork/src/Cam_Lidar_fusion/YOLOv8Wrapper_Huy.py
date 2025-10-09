import cv2
import numpy as np
import torch
from pit.YOLO.nets import YOLOv8  , MASK_COLORS_RGB 
from pit.YOLO.utils import TrafficLight, Obstacle

class EnhancedObstacle(Obstacle):
    """Enhanced Obstacle class with more detailed information"""
    def __init__(self, name='QCar', distance=0, conf=0, x=0, y=0):
        super().__init__(name, distance, conf, x, y)
        # Additional attributes
        self.bbox = None          # [x1, y1, x2, y2]
        self.center_x = None      # Center X coordinate
        self.center_y = None      # Center Y coordinate
        self.width = None         # Bounding box width
        self.height = None        # Bounding box height
        self.area = None          # Bounding box area
        self.class_id = None      # YOLO class ID
        self.mask = None          # Associated segmentation mask
        self.mask_area = None     # Area of the segmentation mask

class EnhancedTrafficLight(TrafficLight):
    """Enhanced TrafficLight class with more detailed information"""
    def __init__(self, color='idle', distance=0):
        super().__init__(color, distance)
        # Additional attributes
        self.bbox = None
        self.center_x = None
        self.center_y = None
        self.width = None
        self.height = None
        self.area = None
        self.class_id = 9  # Traffic light class ID
        self.mask = None          # Associated segmentation mask
        self.mask_area = None     # Area of the segmentation mask

class YOLOv8Wrapper_Huy(YOLOv8):
    """
    Enhanced YOLOv8 wrapper with improved post_processing method
    that provides more detailed detection information including:
    - Full bounding box coordinates
    - Center coordinates
    - Width, height, and area
    - Class ID information
    """
    
    def __init__(self, imageWidth=640, imageHeight=480, modelPath=None):
        """Initialize the enhanced YOLOv8 wrapper"""
        super().__init__(imageWidth, imageHeight, modelPath)
        # print('YOLOv8Wrapper_Huy initialized with enhanced post-processing')
    
    
    def process_qcar_image_with_yolo(self , rgb_image, depth_image=None , apply_fov_alignment =False , kernel_size=5 , iterations=2 , verbose=False):
        """Process RGB image with YOLO (image already obtained from sensors)"""
        if rgb_image is None:
            print("Failed: RGB image is None")
            return None, []
        
        # Run YOLO prediction - focus on cars (class 2)
        prediction = self.predict(
            inputImg=rgb_image,
            classes=[2,9,11],  # Focus only on cars
            confidence=0.3,  # Lower confidence to detect more objects
            half=True,
            verbose=False
        )
        
        # Post-process results with enhanced information
        processedResults = self.post_processing(
                                alignedDepth=depth_image,
                                clippingDistance=5.0,
                                enable_distance_calculation=False,  # Calculate distances
                                apply_fov_alignment=apply_fov_alignment,          # Apply proper FOV alignment
                                morphology_op='dilate', 
                                kernel_size=kernel_size,
                                iterations=iterations,
                                verbose=False
                            )
        
        # Get annotated image with morphological operations applied
        # This ensures masks in processedResults are consistent with rendered masks
        annotatedImg = self.post_process_render(
                            morphology_op='dilate',
                            kernel_size=kernel_size,
                            iterations=iterations,
                        )
        if verbose:
            print(f"YOLO processing: {len(processedResults) if processedResults else 0} objects detected")
        
        return annotatedImg, processedResults


    def visualize_eroded_masks(self, segmentation_masks, image_shape, erosion_kernel_size=3):
        """
        Create a visualization of all eroded segmentation masks combined
        
        Args:
            segmentation_masks: List of segmentation masks
            image_shape: Shape of the target image (height, width)
            erosion_kernel_size: Size of erosion kernel
        
        Returns:
            Combined mask visualization as numpy array
        """
        if not segmentation_masks:
            return np.zeros(image_shape, dtype=np.uint8)
        
        combined_mask = np.zeros(image_shape, dtype=np.uint8)
        erosion_kernel = np.ones((erosion_kernel_size, erosion_kernel_size), np.uint8)
        
        for i, mask in enumerate(segmentation_masks):
            try:
                # Convert mask to numpy array if needed
                if hasattr(mask, 'cpu'):
                    mask_np = mask.cpu().numpy()
                elif hasattr(mask, 'numpy'):
                    mask_np = mask.numpy()
                else:
                    mask_np = np.array(mask)
                
                # Ensure mask is 2D
                if len(mask_np.shape) > 2:
                    mask_np = mask_np.squeeze()
                
                # Resize mask to match image dimensions if needed
                if mask_np.shape != image_shape:
                    mask_np = cv2.resize(mask_np.astype(np.float32), 
                                       (image_shape[1], image_shape[0]), 
                                       interpolation=cv2.INTER_NEAREST)
                
                # Convert to binary mask
                binary_mask = (mask_np > 0.5).astype(np.uint8)
                
                # Apply erosion
                eroded_mask = cv2.erode(binary_mask, erosion_kernel, iterations=1)
                
                # Add to combined mask with unique values for each object
                combined_mask = np.where(eroded_mask > 0, (i + 1) * 50, combined_mask)
                
            except Exception as e:
                print(f"Error visualizing mask {i}: {e}")
                continue
        
        return combined_mask
    
    def extract_masks_from_annotated_image(self, annotated_image, threshold=0.5):
        """
        Extract segmentation masks directly from the annotated image.
        This ensures perfect consistency with what's visually displayed.
        
        Args:
            annotated_image: The annotated image from post_process_render()
            threshold: Threshold for mask detection (default: 0.5)
            
        Returns:
            List of segmentation masks extracted from the annotated image
            Each mask corresponds to detected objects with consistent processing
        """
        masks = []
        
        if not self.processedResults or annotated_image is None:
            return masks
            
        try:
            # Convert to numpy if needed
            if hasattr(annotated_image, 'cpu'):
                img_np = annotated_image.cpu().numpy()
            elif hasattr(annotated_image, 'numpy'):
                img_np = annotated_image.numpy()
            else:
                img_np = np.array(annotated_image)
            
            # Ensure proper format
            if len(img_np.shape) == 3 and img_np.shape[2] == 3:  # RGB image
                # For each detected object, extract its mask based on color
                for i, result in enumerate(self.processedResults):
                    try:
                        # Get the color used for this object
                        color = MASK_COLORS_RGB[self.objectsDetected[i].astype(int)]
                        
                        # Create mask by color matching (with some tolerance)
                        color_tolerance = 30  # Adjust tolerance as needed
                        
                        # Create mask for this specific color
                        mask = np.all(np.abs(img_np - np.array(color)) < color_tolerance, axis=-1)
                        mask = mask.astype(np.uint8)
                        
                        # # Apply morphological cleaning to remove small artifacts
                        # kernel = np.ones((3, 3), np.uint8)
                        # mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
                        # mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
                        
                        # Only keep mask if it has reasonable content
                        if np.sum(mask) > 100:  # Minimum pixels threshold
                            masks.append(mask)
                        else:
                            print(f"Warning: Extracted mask {i} too small, skipping")
                            
                    except Exception as e:
                        print(f"Error extracting mask {i} from annotated image: {e}")
                        continue
                        
        except Exception as e:
            print(f"Error in extract_masks_from_annotated_image: {e}")
            
        return masks

    def extract_segmentation_masks(self, image_height, image_width, morphology_op=None, kernel_size=3, iterations=1):
        """
        Extract segmentation masks from YOLO predictions with optional morphological operations
        OPTIMIZED: Enhanced for LiDAR filtering pipeline compatibility
        
        Args:
            image_height: Height of the target image
            image_width: Width of the target image
            morphology_op: Optional morphological operation ('erode', 'dilate', 'opening', 'closing', None)
            kernel_size: Size of morphological kernel (default: 3)
            iterations: Number of iterations for morphological operation (default: 1)
        
        Returns:
            List of segmentation masks corresponding to detected objects
            Each mask is a 2D numpy array with shape (image_height, image_width)
        """
        masks = []
        
        # Prepare morphological kernel if operation is specified
        kernel = None
        if morphology_op is not None:
            kernel = np.ones((kernel_size, kernel_size), np.uint8)
        
        try:
            if (self.predictions and 
                len(self.predictions) > 0 ):
                
                mask_data = self.predictions[0].masks.data
                
                if len(mask_data) == 0:
                    return masks
                
                # Convert each mask to numpy array and resize if needed
                for i in range(len(mask_data)):
                    try:
                        # Get mask tensor
                        mask_tensor = mask_data[i]
                        
                        # Convert to numpy array
                        if hasattr(mask_tensor, 'cpu'):
                            mask_np = mask_tensor.cpu().numpy()
                        elif hasattr(mask_tensor, 'numpy'):
                            mask_np = mask_tensor.numpy()
                        else:
                            mask_np = np.array(mask_tensor)
                        
                        # Ensure mask is 2D
                        if len(mask_np.shape) > 2:
                            mask_np = mask_np.squeeze()
                        elif len(mask_np.shape) < 2:
                            print(f"Warning: Mask {i} has invalid dimensions: {mask_np.shape}")
                            continue
                        
                        # Resize mask to match image dimensions if needed
                        if mask_np.shape != (image_height, image_width):
                            mask_np = cv2.resize(mask_np.astype(np.float32), 
                                               (image_width, image_height), 
                                               interpolation=cv2.INTER_NEAREST)
                        
                        # Ensure mask is binary (0-1 range)
                        mask_np = (mask_np > 0.5).astype(np.uint8)
                        
                        # Apply morphological operations if specified
                        if morphology_op is not None and kernel is not None:
                            if morphology_op.lower() == 'erode':
                                # Make mask smaller - good for precise object boundaries
                                mask_np = cv2.erode(mask_np, kernel, iterations=iterations)
                            elif morphology_op.lower() == 'dilate':
                                # Make mask bigger - good for safety margins
                                mask_np = cv2.dilate(mask_np, kernel, iterations=iterations)
                            elif morphology_op.lower() == 'opening':
                                # Erode then dilate - removes noise, keeps original size
                                mask_np = cv2.morphologyEx(mask_np, cv2.MORPH_OPEN, kernel, iterations=iterations)
                            elif morphology_op.lower() == 'closing':
                                # Dilate then erode - fills holes, keeps original size
                                mask_np = cv2.morphologyEx(mask_np, cv2.MORPH_CLOSE, kernel, iterations=iterations)
                            else:
                                print(f"Warning: Unknown morphology operation '{morphology_op}'. Using original mask.")
                        
                        # # Convert back to float32 for consistency
                        # mask_np = mask_np.astype(np.float32)
                        
                        # Validate mask has some content
                        if np.sum(mask_np) > 0:
                            masks.append(mask_np)
                        else:
                            print(f"Warning: Mask {i} is empty after processing, skipping")
                        
                    except Exception as e:
                        print(f"Error processing mask {i}: {e}")
                        continue
                        
            else:
                # No segmentation masks available - this is normal for bounding box only models
                pass
                
        except Exception as e:
            print(f"Error extracting segmentation masks: {e}")
        
        return masks

    def render_and_extract_masks(self, showFPS=True, bbox_thickness=4, 
                                morphology_op=None, kernel_size=3, iterations=1):
        """
        Convenient method that renders annotated image and extracts consistent masks.
        
        Args:
            Same as post_process_render()
            
        Returns:
            tuple: (annotated_image, list_of_masks)
                - annotated_image: Rendered image with masks and annotations
                - list_of_masks: Masks extracted from the rendered image (perfectly consistent)
        """
        # Render the annotated image with specified morphology
        annotated_img = self.post_process_render(
            showFPS=showFPS,
            bbox_thickness=bbox_thickness,
            morphology_op=morphology_op,
            kernel_size=kernel_size,
            iterations=iterations
        )
        
        # Extract masks from the rendered image for perfect consistency
        extracted_masks = self.extract_masks_from_annotated_image(annotated_img)
        
        return annotated_img, extracted_masks
    
    
    def debug_detection_object(self, detection, name="Detection"):
        """Debug function to inspect detection object structure"""
        print(f"\n=== {name} Object Debug ===")
        print(f"Type: {type(detection)}")
        print(f"Available attributes: {dir(detection)}")
        
        # Check for YOLOv8Wrapper_Huy attributes
        attrs_to_check = ['center_x', 'center_y', 'width', 'height', 'bbox', 'class_id', 'conf', 'distance', 'name']
        for attr in attrs_to_check:
            if hasattr(detection, attr):
                value = getattr(detection, attr)
                print(f"  {attr}: {value} (type: {type(value)})")
            else:
                print(f"  {attr}: NOT FOUND")
        print("=" * 30)
    
    def post_processing(self, alignedDepth=None, clippingDistance=10, 
                       enable_distance_calculation=True, apply_fov_alignment=True , morphology_op=None, kernel_size=3, iterations=1, verbose=False):
        """
        Enhanced post-processing method with detailed detection information and proper FOV alignment.
        
        Args:
            alignedDepth (ndarray): A depth image aligned to the rgb input
            clippingDistance (float): Pixels in depth image further than this will be set to zero
            verbose (bool): Print debug information
            enable_distance_calculation (bool): Whether to calculate distances using depth data
            apply_fov_alignment (bool): Whether to apply proper FOV alignment between RGB and depth cameras
            
        Returns:
            list: A list of Enhanced Obstacle/TrafficLight objects with detailed information
        """
        self.processedResults = []
        self._calc_distance = False

        if len(self.objectsDetected) == 0:
            if verbose:
                print("No objects detected")
            return self.processedResults
            
        # Get bounding boxes
        self.bounding = self.predictions[0].boxes.xyxy.cpu().numpy().astype(int)
        
        # Check if segmentation masks are available
        masks_available = (self.predictions[0].masks is not None and 
                          len(self.predictions[0].masks.data) > 0)
        
        if verbose:
            print(f"Processing {len(self.objectsDetected)} detected objects")
            if masks_available:
                print(f"Segmentation masks available: {len(self.predictions[0].masks.data)}")
        
        # Prepare depth processing if available and distance calculation is enabled
        if alignedDepth is not None and enable_distance_calculation:
            if alignedDepth.shape[:2] != (self.imageHeight, self.imageWidth):
                alignedDepth = cv2.resize(alignedDepth, (self.imageWidth, self.imageHeight))
            
            # Apply proper FOV alignment if requested (like in apply_rgbd_processing)
            if apply_fov_alignment:
                if verbose:
                    print("Applying FOV alignment between RGB and depth cameras...")
                
                # Calculate scaling factors for current resolution vs original 1280x720
                width_scale = self.imageWidth / 1280.0
                height_scale = self.imageHeight / 720.0
                
                # Original coordinates: [81:618, 108:1132] for 1280x720
                # Adjust the horizontal offset to shift more to the right
                horizontal_offset = 22  # Additional pixels to shift right (adjust as needed)
                
                crop_top = int(81 * height_scale)
                crop_bottom = int(618 * height_scale)
                crop_left = max(0, int(108 * width_scale) - horizontal_offset)
                crop_right = min(self.imageWidth, int(1132 * width_scale) - horizontal_offset)
                
                # Create FOV alignment mask - only the overlapping region is valid for distance calculation
                fov_alignment_mask = np.zeros((self.imageHeight, self.imageWidth), dtype=np.uint8)
                
                # Ensure we have a valid cropping region
                if crop_bottom > crop_top and crop_right > crop_left:
                    # Only the cropped region represents the valid overlap between RGB and Depth FOV
                    fov_alignment_mask[crop_top:crop_bottom, crop_left:crop_right] = 255
                    
                    if verbose:
                        print(f"FOV alignment region: [{crop_top}:{crop_bottom}, {crop_left}:{crop_right}]")
                else:
                    # Fallback: use center region
                    center_y, center_x = self.imageHeight // 2, self.imageWidth // 2
                    margin_y, margin_x = self.imageHeight // 4, self.imageWidth // 4
                    fov_alignment_mask[center_y-margin_y:center_y+margin_y, center_x-margin_x:center_x+margin_x] = 255
                    
                    if verbose:
                        print("Using fallback center region for FOV alignment")
                
                # Apply FOV alignment mask to depth image
                alignedDepth = cv2.bitwise_and(alignedDepth, alignedDepth, mask=fov_alignment_mask)
            
            # Continue with standard depth processing
            depth3D = np.dstack((alignedDepth, alignedDepth, alignedDepth))
            bgRemoved = np.where((depth3D > clippingDistance) | (depth3D <= 0), 0, depth3D)
            self._calc_distance = True
            self.depthTensor = torch.as_tensor(bgRemoved, device="cuda:0")
            
            if verbose:
                valid_depth_pixels = np.sum(bgRemoved[:,:,0] > 0)
                total_pixels = self.imageHeight * self.imageWidth
                print(f"Valid depth pixels after alignment: {valid_depth_pixels}/{total_pixels} ({100*valid_depth_pixels/total_pixels:.1f}%)")
        else:
            if verbose and alignedDepth is not None and not enable_distance_calculation:
                print("Distance calculation disabled - depth data available but not used")
            elif verbose and alignedDepth is None and enable_distance_calculation:
                print("Distance calculation enabled but no depth data provided")
            
        # Process each detection
        for i in range(len(self.objectsDetected)):
            class_id = int(self.objectsDetected[i])
            
            # Get bounding box coordinates
            points = self.predictions[0].boxes.xyxy.cpu()[i]
            bbox = points.numpy().astype(int)  # [x1, y1, x2, y2]
            
            # Calculate center coordinates
            center_x = int((bbox[0] + bbox[2]) / 2)
            center_y = int((bbox[1] + bbox[3]) / 2)
            
            # Calculate dimensions
            width = bbox[2] - bbox[0]
            height = bbox[3] - bbox[1]
            area = width * height
            
            # Get confidence
            conf = self.predictions[0].boxes.conf.cpu().numpy()[i]
            
            if verbose:
                print(f"Object {i+1}: class_id={class_id}, bbox={bbox}, center=({center_x}, {center_y})")
            
            # Create appropriate object type
            if class_id == 9:  # Traffic light
                traffic_box = self.bounding[i]
                traffic_light_color = self.check_traffic_light(traffic_box, self.img)
                result = EnhancedTrafficLight(color=traffic_light_color)
                result.name += (' (' + traffic_light_color + ')')
                result.class_id = 9
            else:
                name = self.predictions[0].names[class_id]
                result = EnhancedObstacle(name=name)
                result.class_id = class_id
            
            # Set enhanced attributes
            result.bbox = bbox.tolist()
            result.center_x = center_x
            result.center_y = center_y
            result.width = width
            result.height = height
            result.area = area
            result.conf = float(conf)
            
            # Legacy attributes for compatibility
            result.x = bbox[0]  # Top-left x (for backward compatibility)
            result.y = bbox[1]  # Top-left y (for backward compatibility)
            
            masks = self.predictions[0].masks.data.cuda()
            # --- Assign segmentation mask with morphology here ---
            # Only if masks are available and index is valid
            if masks_available and i < len(masks):
                try:
                    mask_tensor = masks[i]
                    # Convert to numpy array
                    if hasattr(mask_tensor, 'cpu'):
                        mask_np = mask_tensor.cpu().numpy()
                    elif hasattr(mask_tensor, 'numpy'):
                        mask_np = mask_tensor.numpy()
                    else:
                        mask_np = np.array(mask_tensor)
                    # Ensure mask is 2D
                    if len(mask_np.shape) > 2:
                        mask_np = mask_np.squeeze()
                    # Resize mask to match image dimensions if needed
                    if mask_np.shape != (self.imageHeight, self.imageWidth):
                        mask_np = cv2.resize(mask_np.astype(np.float32), 
                                             (self.imageWidth, self.imageHeight), 
                                             interpolation=cv2.INTER_NEAREST)
                    # Convert to binary mask
                    mask_np = (mask_np > 0.5).astype(np.uint8)
                    # Apply default morphology (dilate, kernel=5x5, 1 iter)
                    kernel = np.ones((kernel_size, kernel_size), np.uint8)
                    mask_np = cv2.dilate(mask_np, kernel, iterations=iterations)
                    result.mask = mask_np
                    result.mask_area = np.sum(mask_np)
                    if verbose:
                        print(f"  Mask assigned: {result.mask_area} pixels")
                except Exception as e:
                    if verbose:
                        print(f"  Mask assignment failed: {e}")
                    result.mask = None
                    result.mask_area = 0
            else:
                result.mask = None
                result.mask_area = 0
                if verbose and not masks_available:
                    print(f"  No segmentation masks available")
            
            # Calculate distance if depth is available and enabled
            if (alignedDepth is not None and enable_distance_calculation and 
                self.predictions[0].masks is not None):
                try:
                    mask = self.predictions[0].masks.data.cuda()[i]
                    distance = self.check_distance(mask, self.depthTensor[:, :, :1])
                    result.distance = distance.cpu().numpy().round(3)
                    if verbose:
                        print(f"  Distance (FOV-aligned): {result.distance}m")
                except Exception as e:
                    if verbose:
                        print(f"  Distance calculation failed: {e}")
                    result.distance = 0
            else:
                result.distance = 0
                if verbose and alignedDepth is not None and not enable_distance_calculation:
                    print(f"  Distance calculation disabled for object {i+1}")
                elif verbose and alignedDepth is None and enable_distance_calculation:
                    print(f"  No depth data available for object {i+1}")
                elif verbose and self.predictions[0].masks is None:
                    print(f"  No segmentation masks available for distance calculation")
                
            self.processedResults.append(result)
            
        if verbose:
            print(f"Enhanced post-processing complete: {len(self.processedResults)} objects processed")
            
        return self.processedResults
    

    def post_process_render(self, showFPS=True, bbox_thickness=4, 
                           morphology_op=None, kernel_size=3, iterations=1):
        '''Annotate input image with colored segmentation mask and distances.
        
        Args:
            showFPS (bool): If set to True, texts showing the FPS will be added 
                            to the top right conner of the output image.
            bbox_thickness (int): Thickness of the bounding box outline.
            morphology_op (str): DEPRECATED - morphological operations are now handled in post_processing
            kernel_size (int): DEPRECATED - morphological operations are now handled in post_processing
            iterations (int): DEPRECATED - morphological operations are now handled in post_processing
        
        Returns:
            ndarray: Input image annotated with colored segmentation masks and
                     distances to the objects.

        '''
        if not self.processedResults:
            if self.img.shape[:2] != self.inputShape:
                out=cv2.resize(self.img,
                               (self.inputShape[1],self.inputShape[0]))
            else: out=self.img
            return out
        
        colors=[]
        boxes = self.predictions[0].boxes.xyxy.cpu().numpy().astype(int)
        imgClone=self.img.copy()
        
        # Extract masks from processedResults (already processed with morphology in post_processing)
        processed_masks = []
        for i in range(len(self.processedResults)):
            if hasattr(self.processedResults[i], 'mask') and self.processedResults[i].mask is not None:
                # Convert numpy mask back to tensor for rendering
                mask_tensor = torch.from_numpy(self.processedResults[i].mask.astype(np.float32)).cuda()
                processed_masks.append(mask_tensor)
            else:
                # Fallback: use original mask if processed mask not available
                if self.predictions[0].masks is not None and i < len(self.predictions[0].masks.data):
                    processed_masks.append(self.predictions[0].masks.data[i])
                else:
                    # Create empty mask as fallback
                    empty_mask = torch.zeros((self.imageHeight, self.imageWidth), dtype=torch.float32).cuda()
                    processed_masks.append(empty_mask)
        
        # Use processed masks for rendering
        if processed_masks:
            masks = torch.stack(processed_masks)
        else:
            # Fallback to original masks if no processed masks available
            masks = self.predictions[0].masks.data.cuda() if self.predictions[0].masks is not None else None
        
        for i in range(len(self.objectsDetected)):
            colors.append(MASK_COLORS_RGB[self.objectsDetected[i].astype(int)])
            # name=self.processedResults[i].name
            x=self.processedResults[i].x
            y=self.processedResults[i].y
            distance=self.processedResults[i].distance
            cv2.rectangle(imgClone,(boxes[i,:2]),(boxes[i,2:4]),colors[i],bbox_thickness)
            # cv2.putText(imgClone, name, 
            #             (x, y-30), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
            #             colors[i], 2)
            if self._calc_distence:
                cv2.putText(imgClone,str(distance) + " m",
                            (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                            colors[i], 2)
        if showFPS:
            cv2.putText(imgClone, 'Inference FPS: '+str(round(self.FPS)), 
                        (470,15), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (255,255,255), 2)
        imgTensor=torch.from_numpy(imgClone).to("cuda:0")
        imgMask=self.mask_color(masks, imgTensor,colors)
        if imgMask.shape[:2] != self.inputShape:
            imgMask=cv2.resize(imgMask,
                               (self.inputShape[1],self.inputShape[0]))
        return imgMask

    def get_detection_summary(self):
        """
        Get a summary of all detections
        
        Returns:
            dict: Summary information about detections
        """
        if not self.processedResults:
            return {"total_detections": 0}
            
        summary = {
            "total_detections": len(self.processedResults),
            "classes": {},
            "total_area": 0,
            "avg_confidence": 0
        }
        
        total_conf = 0
        for obj in self.processedResults:
            class_name = obj.name
            if class_name in summary["classes"]:
                summary["classes"][class_name] += 1
            else:
                summary["classes"][class_name] = 1
                
            summary["total_area"] += obj.area if obj.area else 0
            total_conf += obj.conf if obj.conf else 0
            
        if len(self.processedResults) > 0:
            summary["avg_confidence"] = total_conf / len(self.processedResults)
            
        return summary

    def get_object_mask(self, object_index, morphology_op=None, vertical=True, kernel_size=3, iterations=1):
        """
        Get the segmentation mask for a specific detected object
        
        Args:
            object_index (int): Index of the object in processedResults
            morphology_op (str): Optional morphological operation ('erode', 'dilate', 'opening', 'closing', None)
            kernel_size (int): Size of morphological kernel
            iterations (int): Number of iterations
        
        Returns:
            numpy.ndarray: 2D binary mask for the specified object
        """
        if (not self.processedResults or 
            object_index >= len(self.processedResults) or
            object_index < 0):
            return None
        
        result = self.processedResults[object_index]
        if hasattr(result, 'mask') and result.mask is not None:
            mask = result.mask.copy()
            
            # Apply morphological operations if specified
            if morphology_op is not None:
                kernel = np.ones((kernel_size, kernel_size), np.uint8)
                if vertical:
                    kernel = np.ones((kernel_size, 1), np.uint8)
                if morphology_op.lower() == 'erode':
                    mask = cv2.erode(mask, kernel, iterations=iterations)
                elif morphology_op.lower() == 'dilate':
                    mask = cv2.dilate(mask, kernel, iterations=iterations)
                elif morphology_op.lower() == 'opening':
                    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=iterations)
                elif morphology_op.lower() == 'closing':
                    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=iterations)
                
        return mask
    
    def get_all_object_masks(self, morphology_op=None, kernel_size=3, iterations=1):
        """
        Get segmentation masks for all detected objects
        
        Returns:
            list: List of 2D binary masks, one for each detected object
        """
        masks = []
        for i in range(len(self.processedResults) if self.processedResults else 0):
            mask = self.get_object_mask(i, morphology_op, kernel_size, iterations)
            masks.append(mask)
        return masks
    
    def visualize_object_mask(self, object_index, original_image=None):
        """
        Visualize the mask for a specific object
        
        Args:
            object_index (int): Index of the object
            original_image (numpy.ndarray): Original image to overlay mask on
        
        Returns:
            numpy.ndarray: Image with mask overlay
        """
        mask = self.get_object_mask(object_index)
        if mask is None:
            print(f"No mask available for object {object_index}")
            return original_image if original_image is not None else None
        
        if original_image is None:
            original_image = self.img
        
        # Create colored overlay
        overlay = original_image.copy()
        result_obj = self.processedResults[object_index]
        
        # Use the same color as in detection
        if hasattr(result_obj, 'class_id'):
            color = MASK_COLORS_RGB[result_obj.class_id]
        else:
            color = (0, 255, 0)  # Default green
        
        # Apply mask overlay
        overlay[mask > 0] = color
        
        # Blend with original image
        result_img = cv2.addWeighted(original_image, 0.7, overlay, 0.3, 0)
        
        return result_img