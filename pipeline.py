import pyrealsense2 as rs
import numpy as np
import cv2

robot_color = (255, 0, 0)  # Blue color for the robot point
human_color = (0, 255, 0)  # Green color for the human point

try:
    # Create pipeline
    pipeline = rs.pipeline()

    # Create a config object
    config = rs.config()

    # Tell config that we will use a recorded device from file to be used by the pipeline through playback.
    # rs.config.enable_device_from_file(config, r"D:\Industrial-Robot-Safety\code\robot-safety-detection\f.bag")
    rs.config.enable_device_from_file(config, r"./test_bag.bag")

    # Configure the pipeline to stream the depth color stream
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)


    # Start streaming from file
    pipeline.start(config)

    # Create opencv window to render image in
    cv2.namedWindow("Depth Stream", cv2.WINDOW_AUTOSIZE)

    # Create colorizer object
    colorizer = rs.colorizer()

    # Streaming loop
    for i in range(1):
        # Get frameset of depth
        frames = pipeline.wait_for_frames()

        # Get depth frame
        depth_frame = frames.get_depth_frame()
        # Colorize depth frame to jet colormap
        depth_color_frame = colorizer.colorize(depth_frame)
        # Convert depth_frame to numpy array to render image in opencv
        depth_color_image = np.asanyarray(depth_color_frame.get_data())

        # Get color frame
        color_frame = frames.get_color_frame()
        # Convert color_frame to numpy array to render image in opencv
        color_image = np.asanyarray(color_frame.get_data())

        # pass RGB image to model
        points = [((100, 300), (103, 300))]
        for point in points:
            robot_point = point[0]
            human_point = point[1]
            # get the depth value of the robot and human in cm
            depth_robot = depth_frame.get_distance(robot_point[0], robot_point[1])*100
            depth_human = depth_frame.get_distance(human_point[0], human_point[1])*100
            # visualize point on the image:
            # Visualize points on the images
            cv2.circle(color_image, robot_point, 2, robot_color, -1)
            cv2.circle(color_image, human_point, 2, human_color, -1)
            cv2.circle(depth_color_image, robot_point, 2, robot_color, -1)
            cv2.circle(depth_color_image, human_point, 2, human_color, -1)

            # Display the depth values beside the points
            cv2.putText(color_image, f"{depth_robot:.2f}cm", (robot_point[0] + 30, robot_point[1]+30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, robot_color, 2)
            cv2.putText(color_image, f"{depth_human:.2f}cm", (human_point[0] - 30, human_point[1]-30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, human_color, 2)
            cv2.putText(depth_color_image, f"{depth_robot:.2f}cm", (robot_point[0] + 30, robot_point[1]+30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, robot_color, 2)
            cv2.putText(depth_color_image, f"{depth_human:.2f}cm", (human_point[0] - 30, human_point[1]-30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, human_color, 2)

            # Render images in opencv windows
            cv2.imshow("Depth Stream", depth_color_image)
            cv2.imshow("Color Stream", color_image)

            # Save one frame of depth image and the corresponding RGB image
            cv2.imwrite('saved_depth_image.png', depth_color_image)
            cv2.imwrite('saved_color_image.png', color_image)

        key = cv2.waitKey(1)
        # if pressed escape exit program
        if key == 27:
            cv2.destroyAllWindows()
            break

finally:
    pass