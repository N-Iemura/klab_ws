import cv2
import numpy as np
import tensorflow as tf
import tensorflow_hub as hub

KEYPOINT_THRESHOLD = 0.2

def main():
    model = hub.load('https://tfhub.dev/google/movenet/multipose/lightning/1')
    movenet = model.signatures['serving_default']
    
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    
    while(True):
        ret, frame = cap.read()
        if not ret:
            break

        keypoints_list, scores_list, bbox_list = run_inference(movenet, frame)
        result_image = render(frame, keypoints_list, scores_list, bbox_list)

        cv2.namedWindow("image", cv2.WINDOW_FULLSCREEN)
        cv2.imshow('image', result_image)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
    
    cap.release()
    cv2.destroyAllWindows()

def run_inference(model, image):
    input_image = tf.convert_to_tensor(image)
    input_image = tf.image.resize_with_pad(input_image, 192, 192)
    input_image = tf.expand_dims(input_image, axis=0)
    outputs = model(input_image)
    keypoints = np.squeeze(outputs['output_0'].numpy())

    image_height, image_width = image.shape[:2]
    keypoints_list, scores_list, bbox_list = [], [], []

    for kp in keypoints:
        keypoints = []
        scores = []
        for index in range(17):
            kp_x = int(image_width * kp[index*3+1])
            kp_y = int(image_height * kp[index*3+0])
            score = kp[index*3+2]
            keypoints.append([kp_x, kp_y])
            scores.append(score)
        bbox_ymin = int(image_height * kp[51])
        bbox_xmin = int(image_width * kp[52])
        bbox_ymax = int(image_height * kp[53])
        bbox_xmax = int(image_width * kp[54])
        bbox_score = kp[55]

        keypoints_list.append(keypoints)
        scores_list.append(scores)
        bbox_list.append([bbox_xmin, bbox_ymin, bbox_xmax, bbox_ymax, bbox_score])
    
    return keypoints_list, scores_list, bbox_list

def render(image, keypoints_list, scores_list, bbox_list):
    for keypoints, scores in zip(keypoints_list, scores_list):
        for (x, y), score in zip(keypoints, scores):
            if score > KEYPOINT_THRESHOLD:
                cv2.circle(image, (x, y), 5, (0, 255, 0), -1)
    return image

if __name__ == "__main__":
    main()