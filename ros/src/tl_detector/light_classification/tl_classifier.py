from styx_msgs.msg import TrafficLight
import rospy  # for logwarn messages only
import tensorflow as tf
import numpy as np
import datetime


class TLClassifier(object):
    def __init__(self, sim):
        if sim:
            GRAPH_FILE = 'frozen_inference_graph_sim.pb'
        else:
            GRAPH_FILE = 'frozen_inference_graph_city.pb'
            

        self.graph = tf.Graph()
        self.threshold = 0.25
    
        with self.graph.as_default():
            graph_default = tf.GraphDef()
            with tf.gfile.GFile(GRAPH_FILE, 'rb') as file_id:
                graph_default.ParseFromString(file_id.read())
                tf.import_graph_def(graph_default, name='')
            
            self.image_tensor      = self.graph.get_tensor_by_name('image_tensor:0')
            self.boxes             = self.graph.get_tensor_by_name('detection_boxes:0')
            self.scores            = self.graph.get_tensor_by_name('detection_scores:0')
            self.classes           = self.graph.get_tensor_by_name('detection_classes:0')
            self.num_detections    = self.graph.get_tensor_by_name('num_detections:0')
        
        self.sess = tf.Session(graph=self.graph)    


    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        with self.graph.as_default():
            expanded_image = np.expand_dims(image, axis=0)
            # start = datetime.datetime.now()
            (boxes, scores, classes, num_detections) = self.sess.run(
                [self.boxes, self.scores, self.classes, self.num_detections],
                feed_dict={self.image_tensor:expanded_image})
            # end = datetime.datetime.now()
            # rospy.logwarn( 'Total time %s', (end-start).total_seconds() )
            # rospy.logwarn( 'Scores[0]: %s', scores[0] )
            # rospy.logwarn( 'Scores[0] len: %s', len(scores[0]) )
            # rospy.logwarn( 'Class %s,      Score: %s', classes[0][0], scores[0][0] )
            
            if scores[0][0] > self.threshold:
                if classes[0][0] == 1:
                    # rospy.logwarn('GREEN GREEN GREEN GREEN GREEN')
                    return TrafficLight.GREEN
                elif classes[0][0] == 2:
                    # rospy.logwarn('RED RED RED RED RED')
                    return TrafficLight.RED
                elif classes[0][0] == 3:
                    # rospy.logwarn('YELLOW YELLOW YELLOW YELLOW YELLOW')
                    return TrafficLight.YELLOW
        
        
            # rospy.logwarn('UNKNOWN UNKNOWN UNKNOWN UNKNOWN')
            return TrafficLight.UNKNOWN
