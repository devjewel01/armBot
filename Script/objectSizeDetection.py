import cv2
import numpy as np

class ObjectSizeDetector:
    def __init__(self, camera_index=0):
        self.cap = cv2.VideoCapture(camera_index)
        
        # Set fixed dimensions
        self.WIDTH = 639
        self.HEIGHT = 480
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.HEIGHT)
        
        # Size thresholds
        self.SMALL_THRESHOLD = 17000
        self.MEDIUM_THRESHOLD = 25000
        
        # Minimum area for detection
        self.MIN_AREA = 1200
        
        # Expanded ROI boundaries
        self.ROI_TOP = 100        # Decreased to expand upward
        self.ROI_BOTTOM = 80      # Decreased to expand downward
        self.ROI_LEFT = 20        # Decreased to expand left
        self.ROI_RIGHT = 20       # Decreased to expand right
        
        # Create window with trackbars
        cv2.namedWindow('Settings')
        cv2.createTrackbar('Threshold', 'Settings', 230, 255, lambda x: None)  # Your setting
        cv2.createTrackbar('Blur', 'Settings', 8, 15, lambda x: None)         # Your setting


    def filter_contour(self, contour, frame_height, frame_width):
        """Enhanced contour filtering."""
        area = cv2.contourArea(contour)
        if area < self.MIN_AREA:
            return False
            
        # Get contour's bounding box
        x, y, w, h = cv2.boundingRect(contour)
        
        # Check if contour is within ROI
        if (y < self.ROI_TOP or 
            y + h > frame_height - self.ROI_BOTTOM or
            x < self.ROI_LEFT or 
            x + w > frame_width - self.ROI_RIGHT):
            return False
        
        # Aspect ratio check
        aspect_ratio = float(w)/h if h > 0 else 0
        if aspect_ratio > 4.0 or aspect_ratio < 0.25:
            return False
        
        # Size consistency check
        if w < 20 or h < 20:  # Minimum object dimensions
            return False
            
        return True

    def preprocess_frame(self, frame):
        """Enhanced frame preprocessing with wider ROI."""
        # Ensure frame has correct dimensions
        frame = cv2.resize(frame, (self.WIDTH, self.HEIGHT))
        
        # Get current trackbar values
        thresh_value = cv2.getTrackbarPos('Threshold', 'Settings')
        blur_value = cv2.getTrackbarPos('Blur', 'Settings')
        blur_value = blur_value if blur_value % 2 == 1 else blur_value + 1
        
        # Create ROI mask with wider area
        mask = np.zeros(frame.shape[:2], dtype=np.uint8)
        roi_y1 = self.ROI_TOP
        roi_y2 = self.HEIGHT - self.ROI_BOTTOM
        roi_x1 = self.ROI_LEFT
        roi_x2 = self.WIDTH - self.ROI_RIGHT
        mask[roi_y1:roi_y2, roi_x1:roi_x2] = 255
        
        # Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # Apply ROI mask
        gray = cv2.bitwise_and(gray, gray, mask=mask)
        # Apply Gaussian blur
        blurred = cv2.GaussianBlur(gray, (blur_value, blur_value), 0)
        # Apply thresholding
        _, thresh = cv2.threshold(blurred, thresh_value, 255, cv2.THRESH_BINARY)
        
        return thresh, gray, blurred

    def process_contours(self, frame, contours):
        """Process contours with wider ROI visualization."""
        output = frame.copy()
        frame_height, frame_width = frame.shape[:2]
        
        # Draw ROI rectangle with expanded boundaries
        roi_color = (0, 255, 0)  # Green color for ROI
        roi_thickness = 1        # Thinner line for less intrusion
        cv2.rectangle(output, 
                     (self.ROI_LEFT, self.ROI_TOP), 
                     (frame_width - self.ROI_RIGHT, frame_height - self.ROI_BOTTOM),
                     roi_color, roi_thickness)
        
        for contour in contours:
            if not self.filter_contour(contour, frame_height, frame_width):
                continue
                
            area = cv2.contourArea(contour)
            rect = cv2.minAreaRect(contour)
            box = cv2.boxPoints(rect)
            box = np.array(box, dtype=np.int32)
            
            width_obj = rect[1][0]
            height_obj = rect[1][1]
            aspect_ratio = max(width_obj, height_obj) / min(width_obj, height_obj)
            
            # Classify size
            if area < self.SMALL_THRESHOLD:
                size = "SMALL"
                color = (0, 255, 0)
            elif area < self.MEDIUM_THRESHOLD:
                size = "MEDIUM"
                color = (0, 255, 255)
            else:
                size = "LARGE"
                color = (0, 0, 255)
            
            # Draw contour and measurements
            cv2.drawContours(output, [contour], -1, color, 2)
            cv2.drawContours(output, [box], 0, (255, 0, 0), 2)
            
            M = cv2.moments(contour)
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                
                measurements = [
                    f"Size: {size}",
                    f"Area: {area:.0f}px²",
                    f"Width: {width_obj:.1f}px",
                    f"Height: {height_obj:.1f}px",
                    f"Ratio: {aspect_ratio:.2f}"
                ]
                
                for i, measurement in enumerate(measurements):
                    y_offset = cY - 50 + (i * 20)
                    cv2.putText(output, measurement, (cX - 60, y_offset),
                              cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        
        return output

    def create_preview_strip(self, gray, blurred, thresh):
        """Create the preview strip with processed images."""
        preview_height = 120
        preview_width = self.WIDTH // 3
        
        # Create and convert preview images
        gray_small = cv2.resize(gray, (preview_width, preview_height))
        gray_small = cv2.cvtColor(gray_small, cv2.COLOR_GRAY2BGR)
        
        blurred_small = cv2.resize(blurred, (preview_width, preview_height))
        blurred_small = cv2.cvtColor(blurred_small, cv2.COLOR_GRAY2BGR)
        
        thresh_small = cv2.resize(thresh, (preview_width, preview_height))
        thresh_small = cv2.cvtColor(thresh_small, cv2.COLOR_GRAY2BGR)
        
        # Combine preview images
        preview_strip = np.hstack((gray_small, blurred_small, thresh_small))
        
        # Add labels
        cv2.putText(preview_strip, "Grayscale", (10, 20), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        cv2.putText(preview_strip, "Blurred", (preview_width + 10, 20),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        cv2.putText(preview_strip, "Threshold", (2*preview_width + 10, 20),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        return preview_strip

    def create_display(self, frame, thresh, gray, blurred):
        """Create the complete display output."""
        # Find contours
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Process contours and create main output
        output = self.process_contours(frame, contours)
        
        # Create preview strip
        preview_strip = self.create_preview_strip(gray, blurred, thresh)
        
        # Combine preview strip and main output
        final_output = np.vstack((preview_strip, output))
        
        return final_output

    def run(self):
        """Main detection loop."""
        print("Starting object detection test mode.")
        print("Controls:")
        print("- Adjust 'Threshold' trackbar to fine-tune object detection")
        print("- Adjust 'Blur' trackbar to change noise reduction")
        print("- Press 'q' to quit")
        print("- Press 's' to save current frame")
        
        frame_count = 0
        
        while True:
            ret, frame = self.cap.read()
            if not ret:
                break
            
            # Resize frame to ensure correct dimensions
            frame = cv2.resize(frame, (self.WIDTH, self.HEIGHT))
            
            # Flip frame horizontally for more intuitive display
            frame = cv2.flip(frame, 1)
            
            # Preprocess frame
            thresh, gray, blurred = self.preprocess_frame(frame)
            
            # Create display output
            output = self.create_display(frame, thresh, gray, blurred)
            
            # Display the result
            cv2.imshow('Object Detection Test', output)
            
            # Handle key presses
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('s'):
                filename = f'object_detection_{frame_count}.jpg'
                cv2.imwrite(filename, output)
                print(f"Saved frame as {filename}")
                frame_count += 1
        
        # Cleanup
        self.cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    # Initialize detector
    detector = ObjectSizeDetector(camera_index=4)  # Change camera_index if needed
    
    # Run detection
    detector.run()